/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoaTag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "lpsTdoaTag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"
#include "estimator_kalman.h"


static lpsAlgoOptions_t* options;

static float uwbTdoaDistDiff[LOCODECK_NR_OF_DRONES];

static uint8_t previousdrone;
static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_DRONES];
static dwTime_t arrivals[LOCODECK_NR_OF_DRONES];

static double frameTime_in_cl_A[LOCODECK_NR_OF_DRONES];
static double clockCorrection_T_To_A[LOCODECK_NR_OF_DRONES];

static uint32_t droneStatusTimeout[LOCODECK_NR_OF_DRONES];

#define MEASUREMENT_NOISE_STD 0.15f
#define drone_OK_TIMEOUT 1500

// The maximum diff in distances that we consider to be valid
// Used to sanity check results and remove results that are wrong due to packet loss
#define MAX_DISTANCE_DIFF (5.0f)

static uint32_t statsReceivedPackets = 0;
static uint32_t statsAccepteddroneDataPackets = 0;
static uint32_t statsAcceptedPackets = 0;

static uint64_t timestampToUint64(const uint8_t *ts) {
  dwTime_t timestamp = {.full = 0};
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static uint64_t truncateToTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFFFul;
}

static void enqueueTDOA(uint8_t drone1, uint8_t drone2, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff,

    .dronePosition[0] = options->dronePosition[drone1],
    .dronePosition[1] = options->dronePosition[drone2]
  };

  estimatorKalmanEnqueueTDOA(&tdoa);
}

static double calcClockCorrection(const double frameTime, const double previuosFrameTime) {
    double clockCorrection = 1.0;

    if (frameTime != 0.0) {
      clockCorrection = previuosFrameTime / frameTime;
    }

    return clockCorrection;
}

// The default receive time in the drones for messages from other drones is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static bool isValidRxTime(const int64_t droneRxTime) {
  return droneRxTime != 0;
}

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference drone (Ar), drone n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Referecne drone by drone N expressed in the clock of drone N"
static void rxcallback(dwDevice_t *dev) {
  statsReceivedPackets++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  const uint8_t drone = rxPacket.sourceAddress & 0xff;

  if (drone < LOCODECK_NR_OF_DRONES) {
    const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    const int64_t previous_rxAn_by_T_in_cl_T  = arrivals[drone].full;
    const int64_t rxAn_by_T_in_cl_T  = arrival.full;
    const int64_t previous_txAn_in_cl_An = timestampToUint64(rxPacketBuffer[drone].timestamps[drone]);
    const int64_t txAn_in_cl_An = timestampToUint64(packet->timestamps[drone]);

    if (drone != previousdrone) {
      const int64_t previuos_rxAr_by_An_in_cl_An = timestampToUint64(rxPacketBuffer[drone].timestamps[previousdrone]);
      const int64_t rxAr_by_An_in_cl_An = timestampToUint64(packet->timestamps[previousdrone]);
      const int64_t rxAn_by_Ar_in_cl_Ar = timestampToUint64(rxPacketBuffer[previousdrone].timestamps[drone]);

      if (isValidRxTime(previuos_rxAr_by_An_in_cl_An) && isValidRxTime(rxAr_by_An_in_cl_An) && isValidRxTime(rxAn_by_Ar_in_cl_Ar)) {
        statsAccepteddroneDataPackets++;

        // Caclculate clock correction from drone to reference drone
        const double frameTime_in_cl_An = truncateToTimeStamp(rxAr_by_An_in_cl_An - previuos_rxAr_by_An_in_cl_An);
        const double clockCorrection_An_To_Ar = calcClockCorrection(frameTime_in_cl_An, frameTime_in_cl_A[previousdrone]);

        const int64_t rxAr_by_T_in_cl_T  = arrivals[previousdrone].full;
        const int64_t txAr_in_cl_Ar = timestampToUint64(rxPacketBuffer[previousdrone].timestamps[previousdrone]);

        // Calculate distance diff
        const int64_t tof_Ar_to_An_in_cl_Ar = (((truncateToTimeStamp(rxAr_by_An_in_cl_An - previous_txAn_in_cl_An) * clockCorrection_An_To_Ar) - truncateToTimeStamp(txAr_in_cl_Ar - rxAn_by_Ar_in_cl_Ar))) / 2.0;
        const int64_t delta_txAr_to_txAn_in_cl_Ar = (tof_Ar_to_An_in_cl_Ar + truncateToTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An) * clockCorrection_An_To_Ar);
        const int64_t timeDiffOfArrival_in_cl_Ar =  truncateToTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection_T_To_A[previousdrone] - delta_txAr_to_txAn_in_cl_Ar;

        const float tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_Ar / LOCODECK_TS_FREQ;

        // Sanity check distances in case of missed packages
        if (tdoaDistDiff > -MAX_DISTANCE_DIFF && tdoaDistDiff < MAX_DISTANCE_DIFF) {
          uwbTdoaDistDiff[drone] = tdoaDistDiff;

          enqueueTDOA(previousdrone, drone, tdoaDistDiff);

          statsAcceptedPackets++;
        }
      }
    }

    // Calculate clock correction for tag to drone
    const double frameTime_in_T = truncateToTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);
    frameTime_in_cl_A[drone] = truncateToTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
    clockCorrection_T_To_A[drone] = calcClockCorrection(frameTime_in_T, frameTime_in_cl_A[drone]);

    arrivals[drone].full = arrival.full;
    memcpy(&rxPacketBuffer[drone], rxPacket.payload, sizeof(rangePacket_t));

    droneStatusTimeout[drone] = xTaskGetTickCount() + drone_OK_TIMEOUT;

    previousdrone = drone;
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      setRadioInReceiveMode(dev);
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
      break;
    default:
      ASSERT_FAILED();
  }

  uint32_t now = xTaskGetTickCount();
  options->rangingState = 0;
  for (int drone = 0; drone < LOCODECK_NR_OF_DRONES; drone++) {
    if (now < droneStatusTimeout[drone]) {
      options->rangingState |= (1 << drone);
    }
  }

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
  memset(arrivals, 0, sizeof(arrivals));

  memset(frameTime_in_cl_A, 0, sizeof(frameTime_in_cl_A));
  memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));

  previousdrone = 0;

  memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));

  options->rangingState = 0;
  memset(droneStatusTimeout, 0, sizeof(droneStatusTimeout));

  statsReceivedPackets = 0;
  statsAccepteddroneDataPackets = 0;
  statsAcceptedPackets = 0;
}
#pragma GCC diagnostic pop

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
};


LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d0, &uwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d1, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d2, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d3, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d4, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d5, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d6, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d7, &uwbTdoaDistDiff[7])

LOG_ADD(LOG_UINT32, rxCnt, &statsReceivedPackets)
LOG_ADD(LOG_UINT32, anCnt, &statsAccepteddroneDataPackets)
LOG_ADD(LOG_UINT32, okCnt, &statsAcceptedPackets)

LOG_GROUP_STOP(tdoa)

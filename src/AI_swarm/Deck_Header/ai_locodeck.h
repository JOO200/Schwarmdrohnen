/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * ai_locodeck.h: Dwm1000 deck driver.
 *
 *
 *Modifiziert für AI-Schwarmdrohnen
 */

#ifndef __LOCODECK_H__
#define __LOCODECK_H__

#include <stddef.h>
#include <stdint.h>

#include "libdw1000.h"
#include "stabilizer_types.h"

#define SPEED_OF_LIGHT 299792458.0
// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)


typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
} uwbEvent_t;

#ifndef LOCODECK_NR_OF_DRONES
#define LOCODECK_NR_OF_DRONES 4
#endif

typedef uint64_t locoAddress_t;

typedef enum {
  lpsMode_TWR = 0,
  lpsMode_TDoA = 1
} lpsMode_t;

typedef struct {
  const uint64_t antennaDelay;
  const int rangingFailedThreshold;

  locoAddress_t tagAddress;
  const locoAddress_t droneAddress[LOCODECK_NR_OF_DRONES];		//Drohnenadresse

  // The status of drones. A bit field (bit 0 - drone 0, bit 1 - drone 1 and so on)
  // where a set bit indicates that an drone reentry has been detected
  volatile uint16_t rangingState;

  lpsMode_t rangingMode;

   //TWR data
 
  point_t dronePosition[LOCODECK_NR_OF_DRONES];			
  bool combineddronePositionOk;

  float distance[LOCODECK_NR_OF_DRONES];
  //float pressures[LOCODECK_NR_OF_DRONES];					//Druck für uns nicht relevant? --> Z-Ranger
  int failedRanging[LOCODECK_NR_OF_DRONES];

  // TWR-TDMA options, Time Division Multiple Access
  bool useTdma;
  int tdmaSlot;
} lpsAlgoOptions_t;

point_t* locodeckGetdronePosition(uint8_t drone);

// Callback for one uwb algorithm
typedef struct uwbAlgorithm_s {
  void (*init)(dwDevice_t *dev, lpsAlgoOptions_t* options);
  uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
} uwbAlgorithm_t;

#include <FreeRTOS.h>

#define MAX_TIMEOUT portMAX_DELAY

// Send a short configuration packet to the LPS system
// Returns true if packet will be send, false instead
bool lpsSendLppShort(uint8_t destId, void* data, size_t length);

typedef struct {
  uint8_t dest;
  uint8_t length;
  uint8_t data[30];
} lpsLppShortPacket_t;

// Poll if there is a LPS short configuration packet to send
// Return true if the packet data has been filled in shortPacket
// Return false if no packet to send
// Function to be used by the LPS algorithm
bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket);

// Handle incoming short LPP packets from the UWB system
void lpsHandleLppShortPacket(uint8_t srcId, uint8_t *data, int length);

// LPP Packet types and format
#define LPP_HEADER_SHORT_PACKET 0xF0

#define LPP_SHORT_dronePOS 0x01

struct lppShortdronePos_s {
  float x;
  float y;
  float z;
} __attribute__((packed));

#endif // __LOCODECK_H__

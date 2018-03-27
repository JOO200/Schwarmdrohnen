/*
 ai_dwm1000_driver.c

 Diese Datei definiert die Funktionen, welche benoetigt werden, um den DWM1000 zu steuern (Daten Senden, Ranging, ...).
 Zusaetzlich werden die wichtigen Registerbkonfiguriert.

 Sollten keine Funktionsaenderungen des DWM1000 gewuenscht sein, sollte dieser Driver am Besten NICHT geaendert werden!
 */

#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../../../vendor/CMSIS/CMSIS/Driver/Include/Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
#include "stm32f4xx_exti.h"	//wird benoetigt um externe interrupts zu initialisieren
#include "../ai_datatypes.h"
#include "stm32f4xx.h"
#include "../ai_task.h"
#include <stdlib.h>
#include "mac.h"
#include "ledring12.h"
#include "stm32f4xx_gpio.h"
#include "semphr.h"
#include "lpsTwrTag.h"
#include "locodeck.h"
#include "lpsTdma.h"

//Hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;
bool DWM1000_IRQ_FLAG = 0;
static bool ai_ranging_complete = false;
static bool ai_lpp_transaction = false;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static st_message_t txPacket;
static volatile uint8_t curr_seq = 0;

static int currentRanging;
static bool tdmaSynchronized;

#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
// Rangin statistics
//static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
//static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values
static uint8_t succededRanging[LOCODECK_NR_OF_ANCHORS];
static uint8_t failedRanging[LOCODECK_NR_OF_ANCHORS];

static lpsLppShortPacket_t lppShortPacket;
lpsAlgoOptions_t * options;

static dwTime_t frameStart;

static void ai_txCallback(dwDevice_t * dev) {
	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (options->antennaDelay / 2);

	switch (txPacket.payload[0]) {
	case LPS_TWR_POLL:
		poll_tx = departure;
		break;
	case LPS_TWR_FINAL:
		final_tx = departure;
		break;
	}
}

static uint32_t ai_rxCallback(dwDevice_t * dev) {
	//dwTime_t arival = { .full = 0 };
	int dataLength = dwGetDataLength(dev);

	if (dataLength == 0)
		return MAX_TIMEOUT ;

	st_message_t message;
	memset(&message, 0, sizeof(st_message_t));

	dwGetData(dev, (uint8_t*) &message, dataLength);
	ai_showDistance(100.0f);
	if (message.destAddress != options->tagAddress) {// Die Nachricht geht nicht an uns.
		return MAX_TIMEOUT ;
	}

	switch (message.messageType) {
	case DISTANCE_REQUEST:

		txPacket.destAddress = message.sourceAddress;
		txPacket.sourceAddress = message.destAddress;

		ai_showDistance(100.0f);
		/*
		switch (message.payload[LPS_TWR_TYPE]) {
		case LPS_TWR_ANSWER:
			if (message.payload[LPS_TWR_SEQ] != curr_seq) {
				return 0;
			}
			if (dataLength - MAC802154_HEADER_LENGTH > 3) {
				if (message.payload[LPS_TWR_LPP_HEADER]
						== LPP_HEADER_SHORT_PACKET) {
					int srcId = -1;

					for (int i = 0; i < LOCODECK_NR_OF_ANCHORS; i++) {
						if (message.sourceAddress
								== options->anchorAddress[i]) {
							srcId = i;

							break;
						}
					}

					if (srcId >= 0) {
						lpsHandleLppShortPacket(srcId,
								&message.payload[LPS_TWR_LPP_TYPE],
								dataLength - MAC802154_HEADER_LENGTH - 3);
					}
				}
			}

			txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
			txPacket.payload[LPS_TWR_SEQ] = message.payload[LPS_TWR_SEQ];

			dwGetReceiveTimestamp(dev, &arival);
			arival.full -= (options->antennaDelay / 2);
			answer_rx = arival;
			dwNewTransmit(dev);
			dwSetData(dev, (uint8_t*) &txPacket, MAC802154_HEADER_LENGTH + 2);
			dwWaitForResponse(dev, true);
			dwStartTransmit(dev);
			break;
		case LPS_TWR_REPORT: {
			lpsTwrTagReportPayload_t *report =
					(lpsTwrTagReportPayload_t *) (message.payload + 2);
			double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

			if (message.payload[LPS_TWR_SEQ] != curr_seq) {
				return 0;
			}

			memcpy(&poll_rx, &report->pollRx, 5);
			memcpy(&answer_tx, &report->answerTx, 5);
			memcpy(&final_rx, &report->finalRx, 5);

			tround1 = answer_rx.low32 - poll_tx.low32;
			treply1 = answer_tx.low32 - poll_rx.low32;
			tround2 = final_rx.low32 - answer_tx.low32;
			treply2 = final_tx.low32 - answer_rx.low32;

			tprop_ctn = ((tround1 * tround2) - (treply1 * treply2))
					/ (tround1 + tround2 + treply1 + treply2);

			tprop = tprop_ctn / LOCODECK_TS_FREQ;
			options->distance[currentRanging] = SPEED_OF_LIGHT * tprop;
			options->pressures[currentRanging] = report->asl;

			// Outliers rejection
			rangingStats[currentRanging].ptr = (rangingStats[currentRanging].ptr
					+ 1) % RANGING_HISTORY_LENGTH;
			float32_t mean;
			float32_t stddev;

			arm_std_f32(rangingStats[currentRanging].history,
					RANGING_HISTORY_LENGTH, &stddev);
			arm_mean_f32(rangingStats[currentRanging].history,
					RANGING_HISTORY_LENGTH, &mean);
			float32_t diff = fabsf(mean - options->distance[currentRanging]);

			rangingStats[currentRanging].history[rangingStats[currentRanging].ptr] =
					options->distance[currentRanging];

			if ((options->combinedAnchorPositionOk
					|| options->anchorPosition[currentRanging].timestamp)
					&& (diff < (OUTLIER_TH * stddev))) {
				distanceMeasurement_t dist;
				dist.distance = options->distance[currentRanging];
				dist.x = options->anchorPosition[currentRanging].x;
				dist.y = options->anchorPosition[currentRanging].y;
				dist.z = options->anchorPosition[currentRanging].z;
				dist.stdDev = 0.25;
				estimatorKalmanEnqueueDistance(&dist);
			}

			if (options->useTdma && currentRanging == 0) {
				// Final packet is sent by us and received by the anchor
				// We use it as synchonisation time for TDMA
				dwTime_t offset = { .full = final_tx.full - final_rx.full };
				frameStart.full = TDMA_LAST_FRAME(final_rx.full) + offset.full;
				tdmaSynchronized = true;
			}

			ai_ranging_complete = true;

			return 0;
			break;
		}
		}*/
		break;
	default:
		break;
		return MAX_TIMEOUT ;
	}
	return MAX_TIMEOUT;
}

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 */
static uint32_t adjustTxRxTime(dwTime_t *time)
{
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));
  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);
  return added;
}

void initiateRanging(dwDevice_t * dev) {
	if(!options->useTdma || tdmaSynchronized) {
		if(options->useTdma) {
			frameStart.full += TDMA_FRAME_LEN;
		}
		currentRanging ++;
		if(currentRanging >= LOCODECK_NR_OF_ANCHORS) {
			currentRanging = 0;
		}
	} else currentRanging = 0;

	dwIdle(dev);

	txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
	txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;

	txPacket.sourceAddress = options->tagAddress;
	  txPacket.destAddress = options->anchorAddress[currentRanging];

	  dwNewTransmit(dev);
	  dwSetDefaults(dev);
	  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

	  if (options->useTdma && tdmaSynchronized) {
	    dwTime_t txTime = {.full = 0};
	    txTime.full = frameStart.full + options->tdmaSlot*TDMA_SLOT_LEN;
	    adjustTxRxTime(&txTime);
	    dwSetTxRxTime(dev, txTime);
	  }

	  dwWaitForResponse(dev, true);
	  dwStartTransmit(dev);
}

	void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet) {
		dwIdle(dev);

		txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_LPP_SHORT;
		memcpy(&txPacket.payload[LPS_TWR_SEND_LPP_PAYLOAD], packet->data,
				packet->length);

		txPacket.sourceAddress = options->tagAddress;
		txPacket.destAddress = options->anchorAddress[packet->dest];

		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*) &txPacket,
				MAC802154_HEADER_LENGTH + 1 + packet->length);

		dwWaitForResponse(dev, false);
		dwStartTransmit(dev);
	}

	static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
		options = algoOptions;

		// Initialize the packet in the TX buffer
		memset(&txPacket, 0, sizeof(txPacket));
		// MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
		txPacket.distance_measurement_s.fcf_s.type = MAC802154_TYPE_DATA;
		txPacket.distance_measurement_s.fcf_s.security = 0;
		txPacket.distance_measurement_s.fcf_s.framePending = 0;
		txPacket.distance_measurement_s.fcf_s.ack = 0;
		txPacket.distance_measurement_s.fcf_s.ipan = 1;
		txPacket.distance_measurement_s.fcf_s.destAddrMode = 3;
		txPacket.distance_measurement_s.fcf_s.version = 1;
		txPacket.distance_measurement_s.fcf_s.srcAddrMode = 3;
		txPacket.pan = 0xbccf;

		memset(&poll_tx, 0, sizeof(poll_tx));
		memset(&poll_rx, 0, sizeof(poll_rx));
		memset(&answer_tx, 0, sizeof(answer_tx));
		memset(&answer_rx, 0, sizeof(answer_rx));
		memset(&final_tx, 0, sizeof(final_tx));
		memset(&final_rx, 0, sizeof(final_rx));

		curr_seq = 0;
		currentRanging = 0;

		options->rangingState = 0;
		ai_ranging_complete = false;

		tdmaSynchronized = false;

		memset(options->distance, 0, sizeof(options->distance));
		memset(options->pressures, 0, sizeof(options->pressures));
		memset(options->failedRanging, 0, sizeof(options->failedRanging));
	}

	static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
		static uint32_t statisticStartTick = 0;
		if (statisticStartTick == 0) {
			statisticStartTick = xTaskGetTickCount();
		}

		switch (event) {
		case eventPacketReceived:
			ai_rxCallback(dev);
			dwNewReceive(dev);
			dwSetDefaults(dev);
			dwStartReceive(dev);
			break;
		case eventPacketSent:
			ai_txCallback(dev);

			if (ai_lpp_transaction) {
				return 0;
			}
			return MAX_TIMEOUT ;
		case eventTimeout:
			if (!ai_ranging_complete && !ai_lpp_transaction) {
				options->rangingState &= ~(1 << currentRanging);
				if (options->failedRanging[currentRanging]
						< options->rangingFailedThreshold) {
					options->failedRanging[currentRanging]++;
					options->rangingState |= (1 << currentRanging);
				}
				//locSrvSendRangeFloat(currentRanging, 0.0/0.0);
				failedRanging[currentRanging]++;
			} else {
				options->rangingState |= (1 << currentRanging);
				options->failedRanging[currentRanging] = 0;
				//locSrvSendRangeFloat(currentRanging,
				//		options->distance[currentRanging]);
				succededRanging[currentRanging]++;
			}
			if (lpsGetLppShort(&lppShortPacket)) {
				ai_lpp_transaction = true;
				sendLppShort(dev, &lppShortPacket);
			} else {
				ai_lpp_transaction = false;
				ai_ranging_complete = false;
				initiateRanging(dev);
			}
			return MAX_TIMEOUT ;
			break;
		case eventReceiveTimeout:
			dwNewReceive(dev);
			dwSetDefaults(dev);
			dwStartReceive(dev);
			break;
		default:
			ASSERT_FAILED();
		}
		return MAX_TIMEOUT ;
	}

uwbAlgorithm_t aiuwbTdoaTagAlgorith =
	{ 	.init = Initialize,
		.onEvent = onEvent,
	};


	bool dwm1000_SendData(st_message_t *message) {
		//spiSetSpeed(dwm, dwSpiSpeedLow);

		dwNewTransmit(dwm);
		dwSetData(dwm, (uint8_t*) message, sizeof(st_message_t));

		dwStartTransmit(dwm);

		vTaskDelay(M2T(3));

		dwNewReceive(dwm);
		dwStartReceive(dwm);

		return 1;
	}
/*
	void dwm1000_ReceiveData(st_message_t *data) {

		unsigned int dataLength = dwGetDataLength(dwm);

		if (dataLength == 0) {
			DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
			return;		//if Fall fuer leere Empfangsdaten
		}
		if (dataLength > sizeof(st_message_t)) {
			DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
			return;
		}
		dwGetData(dwm, (uint8_t*) data, dataLength);//get Packet und befuellen

		//return data->messageType;

	}*/

//---------------------- RANGING ----------------------

	/*
	 Sendet Distanzrequest an spezifiziertes Target

	 // Beschreibung des gesammten Ranginvorgangs:
	 //1. Nachrichten an Partner schicken (requester)
	 Flag setzen: requestTransmitTimestampPending

	 //2. Partner antwortet  direkt (target)
	 Flag setzen: immediateAnswerTransmitTimestampPending

	 //3. Master empfägt target-Nachricht - berechnet T_round (Zeit die die Nachricht hin und zurück gebraucht hat)
	 Flag setzen: processingTimePending

	 //4. target berechnet seine Bearbeitungszeit (immediateAnswerTranceiveTimestamp - requestReceiveTimestamp)

	 //5. (Gestoppte Zeit - (Bearbeitungstimestamp))/2 * Lichtgeschw = Abstand

	 // Danach weiß der requester den Abstand
	 *//*
	void dwm1000_requestDistance(char targetID) {
		st_message_t requestMessage;
		requestMessage.senderID = AI_NAME;
		requestMessage.targetID = targetID;
		requestMessage.messageType = DISTANCE_REQUEST;

		dwm1000_SendData(&requestMessage);
	}*/
	/*
	 //1. Funktion aktiviert, nach Distance request Eingang
	 //2. Antworten, damit requester Zeit stopppen kann

	 //Zusatz: Zeit zwischen Receive Timestamp und Transmit Timestamp an requester schicken, damit von gestoppter Zeit abgezogen werden kann
	 //Receive Timestamp - Transmit Timestamp
	 *//*
	void dwm1000_immediateDistanceAnswer(char id_requester) {
		st_message_t immediateAnswer;
		immediateAnswer.senderID = AI_NAME;
		immediateAnswer.targetID = id_requester;
		immediateAnswer.messageType = IMMEDIATE_ANSWER;

		dwm1000_SendData(&immediateAnswer);
	}*/
/*
	void dwm1000_sendProcessingTime(char id_requester) {

		//1. Timestamp TX lesen, Register 0x17 Timestamp von bit 0-39

		dwTime_t txTimeStamp;
		dwGetTransmitTimestamp(dwm, &txTimeStamp);

		//2. Timestamp RX lesen, Register 0x15 Timestamp von bit 0-39
		dwTime_t rxTimeStamp;
		dwGetReceiveTimestamp(dwm, &rxTimeStamp);

		dwTime_t processingTime;
		processingTime.full = rxTimeStamp.full - txTimeStamp.full;

		//4. an requester schicken

		st_message_t processingTimeMessage;
		processingTimeMessage.targetID = id_requester;
		processingTimeMessage.st_distance_broadcast_s.time = processingTime;
		processingTimeMessage.messageType = PROCESSING_TIME;

		dwm1000_SendData(&processingTimeMessage);
	}*/

	dwTime_t dwm1000_getTxTimestamp() {
		dwTime_t ret;
		dwGetTransmitTimestamp(dwm, &ret);

		return ret;
	}

	dwTime_t dwm1000_getRxTimestamp() {
		dwTime_t ret;
		dwGetTransmitTimestamp(dwm, &ret);

		return ret;
	}
/*
static e_interrupt_type_t dwm1000_EvalInterrupt() {
		//zurücksetzen
		e_interrupt_type_t lastInterrupt = FAILED_EVAL;

		//funktionen callen, welche je nach statusregisterzustand entsprechende handler callt
		dwHandleInterrupt(dwm);

		//von handlern gesetzte werte zurückgeben
		return lastInterrupt;
	}*/

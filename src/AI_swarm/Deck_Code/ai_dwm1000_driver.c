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

void initiateAiRanging(dwDevice_t * dev) {
	dwIdle(dev);

	txPacket.messageType = DISTANCE_REQUEST;
	txPacket.sourceAddress = 2;
	txPacket.destAddress = 1;
	txPacket.distance_measurement_s.receiveTimestamp.full = 0;
	txPacket.distance_measurement_s.sendTimestamp.full = 0;

	//dwm1000_SendData(&txPacket);
	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*) &txPacket, sizeof(st_message_t));

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);
}

static void ai_txCallback(dwDevice_t * dev) {
	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);

	if (txPacket.messageType == IMMEDIATE_ANSWER) {
		txPacket.messageType = PROCESSING_TIME;
		txPacket.distance_measurement_s.sendTimestamp = departure;
		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*) &txPacket, sizeof(st_message_t));

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);
		ai_showDistance(50, 0x10, 0, 0);
	} else if (txPacket.messageType == DISTANCE_REQUEST) {
		poll_tx = departure;
	}
}

static uint32_t ai_rxCallback(dwDevice_t * dev) {
	dwTime_t arival = { .full = 0 };
	int dataLength = dwGetDataLength(dev);

	if (dataLength == 0)
		return MAX_TIMEOUT ;

	st_message_t message;
	memset(&message, 0, sizeof(st_message_t));

	dwGetData(dev, (uint8_t*) &message, dataLength);
	/*if (message.destAddress != options->tagAddress) {// Die Nachricht geht nicht an uns.
	 return MAX_TIMEOUT ;
	 }*/
	//ai_showDistance(100, 0, 0x10, 0x00);
	//ai_showDistance(message.messageType * 10);
	switch (message.messageType) {
	case IMMEDIATE_ANSWER:
		dwGetReceiveTimestamp(dev, &arival);
		answer_rx = arival;
		ai_showDistance(50, 0, 0x10, 0x10);
		break;
	case DISTANCE_REQUEST:

		ai_showDistance(50, 0x10, 0x10, 0x10);
		dwGetReceiveTimestamp(dev, &arival);
		txPacket.messageType = IMMEDIATE_ANSWER;
		txPacket.destAddress = message.sourceAddress;
		txPacket.sourceAddress = message.destAddress;
		txPacket.distance_measurement_s.receiveTimestamp = arival;
		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*) &txPacket, sizeof(st_message_t));
		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);
		break;
	case PROCESSING_TIME:
		dwGetReceiveTimestamp(dev, &arival);
		/*dwTime_t sendingTime = (arival-frameStart) -
		 (message->distance_measurement_s.sendTimestamp - message->distance_measurement_s.ReceiveTimestamp);*/
		double tround1 = message.distance_measurement_s.sendTimestamp.low32
				- message.distance_measurement_s.receiveTimestamp.low32;
		double tround2 = answer_rx.low32 - poll_tx.low32;

		double tdiff = (tround2 - tround1) / 2;
		double distance = SPEED_OF_LIGHT * tdiff; // Umrechnung in m
		ai_showDistance(distance, 0x00, 0x00, 0x10);
		initiateAiRanging(dev);
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
	return MAX_TIMEOUT ;
}

static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {

	// Initialize the packet in the TX buffer
	memset(&txPacket, 0, sizeof(txPacket));
	// MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

	memset(&poll_tx, 0, sizeof(poll_tx));
	memset(&poll_rx, 0, sizeof(poll_rx));
	memset(&answer_tx, 0, sizeof(answer_tx));
	memset(&answer_rx, 0, sizeof(answer_rx));
	memset(&final_tx, 0, sizeof(final_tx));
	memset(&final_rx, 0, sizeof(final_rx));

	curr_seq = 0;
	currentRanging = 0;

	ai_ranging_complete = false;

	tdmaSynchronized = false;

	initiateAiRanging(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {

	//ai_showDistance(event, 0x0, 0, 0x10);
	switch (event) {
	case eventPacketReceived:
		ai_rxCallback(dev);
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
		break;
	case eventPacketSent:
		ai_txCallback(dev);
		return MAX_TIMEOUT ;
	case eventTimeout:

		//ai_showDistance(20, 0x10, 0x00, 0x00);
		//initiateAiRanging(dev);
		return MAX_TIMEOUT ;
		break;
	case eventReceiveTimeout:
		//ai_showDistance(20, 0x10, 0x00, 0x10);
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
		//initiateAiRanging(dev);
		break;
	default:
		ASSERT_FAILED();
	}
	return MAX_TIMEOUT ;
}

uwbAlgorithm_t aiuwbTdoaTagAlgorith =
		{ .init = Initialize, .onEvent = onEvent, };

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

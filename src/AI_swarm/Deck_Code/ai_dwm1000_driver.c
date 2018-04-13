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
#include "ai_manageInterrupts.h"

//Hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;
bool DWM1000_IRQ_FLAG = 0;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static st_message_t txPacket;

#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
// Rangin statistics
//static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
//static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values

void sendData(dwDevice_t * dev, st_message_t * message) {
	//spiSetSpeed(dev, dwSpiSpeedLow);
	vTaskDelay(M2T(10));
	dwNewTransmit(dev);
	dwSetData(dwm, (uint8_t*)message, sizeof(*message));
	dwWaitForResponse(dwm, true);
	dwStartTransmit(dwm);
	vTaskDelay(M2T(3)); //TO
	dwNewReceive(dwm);
	dwStartReceive(dwm);

}

void initiateAiRanging(dwDevice_t * dev) {
	//dwIdle(dev);

	txPacket.messageType = DISTANCE_REQUEST;
	txPacket.sourceAddress = 2;
	txPacket.destAddress = 1;
	txPacket.distance_measurement_s.receiveTimestamp.full = 0;
	txPacket.distance_measurement_s.sendTimestamp.full = 0;

	sendData(dev, &txPacket);
}

static void ai_txCallback(dwDevice_t * dev) {
	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);

	if (txPacket.messageType == IMMEDIATE_ANSWER) {
		answer_rx = departure;
	} else if (txPacket.messageType == DISTANCE_REQUEST) {
		poll_tx = departure;
	}
}

static uint32_t ai_rxCallback(dwDevice_t * dev) {
	dwTime_t arival = { .full = 0 };
	int dataLength = dwGetDataLength(dev);

	ai_showDistance(100, 0x10, 0, 0x10);
	sendData(dev, &txPacket);

	//if (dataLength == 0)
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
		memcpy(&txPacket, &message, sizeof(st_message_t));
		answer_rx = arival;
		txPacket.messageType = IMMEDIATE_ANSWER_ACK;
		sendData(dev, &txPacket);
		break;
	case DISTANCE_REQUEST:
		dwGetReceiveTimestamp(dev, &arival);
		memcpy(&txPacket, &message, sizeof(st_message_t));
		txPacket.messageType = IMMEDIATE_ANSWER;
		txPacket.distance_measurement_s.receiveTimestamp = arival;
		sendData(dev, &txPacket);
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
		break;
	case IMMEDIATE_ANSWER_ACK:
		dwGetReceiveTimestamp(dev, &arival);
		memcpy(&txPacket, &message, sizeof(st_message_t));
		txPacket.messageType = PROCESSING_TIME;
		txPacket.distance_measurement_s.sendTimestamp = answer_rx;
		sendData(dev, &txPacket);
		//setAckAnswerTxTs(message.senderID, arival);
		break;
	default:
		break;
	}
	return MAX_TIMEOUT ;
}

static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
	memset(&txPacket, 0, sizeof(txPacket));
	dwm = dev;
	// MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

	memset(&poll_tx, 0, sizeof(poll_tx));
	memset(&poll_rx, 0, sizeof(poll_rx));
	memset(&answer_tx, 0, sizeof(answer_tx));
	memset(&answer_rx, 0, sizeof(answer_rx));
	memset(&final_tx, 0, sizeof(final_tx));
	memset(&final_rx, 0, sizeof(final_rx));

	initiateAiRanging(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {

	//ai_showDistance(event, 0x0, 0, 0x10);
	switch (event) {
	case eventPacketReceived:
		ai_rxCallback(dev);
		break;
	case eventPacketSent:
		ai_txCallback(dev);
		break;
	case eventTimeout:

		//ai_showDistance(20, 0x10, 0x00, 0x00);
		initiateAiRanging(dev);
		break;
	case eventReceiveTimeout:
		//ai_showDistance(20, 0x10, 0x00, 0x10);
		initiateAiRanging(dev);
		break;
	default:
		ASSERT_FAILED();
	}
	return 500000 ;
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

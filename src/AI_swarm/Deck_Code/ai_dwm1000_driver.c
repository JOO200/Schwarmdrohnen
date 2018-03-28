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

static st_message_t txPacket;

#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
// Rangin statistics
//static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
//static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values
void initiateAiRanging(uint8_t target) {
	dwDevice_t *dev = dwm;
	dwIdle(dev);

	txPacket.messageType = DISTANCE_REQUEST;
	txPacket.senderID = AI_NAME;
	txPacket.targetID = target;
	txPacket.time.full = 0;

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
		setImmediateAnswerTxTs(txPacket.targetID, departure);
	} else if (txPacket.messageType == DISTANCE_REQUEST) {
		setReqTxTs(txPacket.targetID, departure);
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
		setImmediateAnswerRxTs(message.senderID, arival);
		break;
	case DISTANCE_REQUEST:
		dwGetReceiveTimestamp(dev, &arival);
		setReqRxTs(message.senderID, arival);
		break;
	case PROCESSING_TIME:
		dwGetReceiveTimestamp(dev, &arival);
		setProcessingTime(message.senderID, message.time);

		break;
	case IMMEDIATE_ANSWER_ACK:
		dwGetReceiveTimestamp(dev, &arival);
		setAckAnswerTxTs(message.senderID, arival);
		break;
	default:
		break;
	}
	return MAX_TIMEOUT ;
}

static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
	memset(&txPacket, 0, sizeof(txPacket));
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {

	//ai_showDistance(event, 0x0, 0, 0x10);
	switch (event) {
	case eventPacketReceived:
		ai_rxCallback(dev);
		dwNewReceive(dev);
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
		//initiateAiRanging(dev);
		dwNewReceive(dev);
		dwStartReceive(dev);
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

	memcpy(message, &txPacket, sizeof(st_message_t));

	dwStartTransmit(dwm);

	vTaskDelay(M2T(3));

	dwNewReceive(dwm);
	dwStartReceive(dwm);

	return 1;
}

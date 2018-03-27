#include "ai_manageInterrupts.h"
#include "ai_datatypes.h"
//Hier werden die Ergebnisse der Reaktionen auf Interrupts des DWM1000 festgehalten


//------Request

//sieht nach ob neuer DistanceRequestRxTs von Drohne mit id "ID" da ist


//------Request RX Timestamp
bool requestRxTsNew[NR_OF_DRONES];
bool lookForRequestRxTs(uint8_t ID){
	if (requestRxTsNew[ID]){
		requestRxTsNew[ID] = false;
		return true;
	}
	return false;
}
dwTime_t reqRxTs[NR_OF_DRONES];
dwTime_t getReqRxTs(uint8_t ID){
	return reqRxTs[ID];
}

dwTime_t setReqRxTs(uint8_t ID, dwTime_t reqRxTsInput){
	reqRxTs[ID] = reqRxTsInput;
	requestRxTsNew[ID] = true;
}

//------Request TX Timestamp
bool requestTxTsNew[NR_OF_DRONES];
bool lookForRequestTxTs(uint8_t ID){
	if (requestTxTsNew[ID]){
		requestTxTsNew[ID] = false;
		return true;
	}
	return false;
}

dwTime_t reqTxTs[NR_OF_DRONES];
dwTime_t getReqTxTs(uint8_t ID){
	return reqTxTs[ID];
}

dwTime_t setReqTxTs(uint8_t ID, dwTime_t reqTxTsInput){
	reqTxTs[ID] = reqTxTsInp;
	requestTxTsNew[ID] = true;
}


//------Immediate Answer RX Timestamp
bool immediateAnswerRxTsNew[NR_OF_DRONES];
bool lookForImmedatateAnswerRxTs(uint8_t ID){
	if (immediateAnswerRxTsNew[ID]){
		immediateAnswerRxTsNew[ID] = false;
		return true;
	}
	return false;
}

dwTime_t immediateAnswerRxTs[NR_OF_DRONES];
dwTime_t getImmediateAnswerRxTs(uint8_t ID){
	return immediateAnswerRxTs;
}

dwTime_t setImmediateAnswerRxTs(uint8_t ID, dwTime_t immAnsRxTs){
	immediateAnswerRxTs[ID] = immAnsRxTs;
	immediateAnswerRxTs[ID] = true;
}


//------Immediate Answer TX Timestamp
bool immediateAnswerTxTsNew[NR_OF_DRONES];
bool lookForImmediatateAnswerTxTs(uint8_t ID){
	if (immediateAnswerTxTsNew[ID]){
		immediateAnswerTxTsNew[ID] = false;
		return true;
	}
	return false;
}

dwTime_t immediateAnswerTxTs[NR_OF_DRONES];
dwTime_t getImmediateAnswerTxTs(uint8_t ID){
	return immediateAnswerTxTs;
}

void setImmediateAnswerTxTs(uint8_t ID, dwTime_t immAnsTxTs){
	immediateAnswerTxTs[ID] = immAnsTxTs;
	immediateAnswerTxTsNew[ID] = true;
}


//------Processing Time
bool processingTimeNew[NR_OF_DRONES];
bool lookForProcessingTime(uint8_t ID){
	if (processingTimeNew[ID]){
		processingTimeNew[ID] = false;
		return true;
	}
	return false;
}

dwTime_t processingTime[NR_OF_DRONES];
dwTime_t getProcessingTime(uint8_t ID){
	return processingTime;
}

void setProessingTime(uint8_t ID, dwTime_t procTime){
	processingTime[ID] = procTime;
	processingTimeNew[ID] = true;
}


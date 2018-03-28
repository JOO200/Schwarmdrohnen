#ifndef _AI_MANAGEINTS_
#define _AI_MANAGEINTS_

#include "ai_datatypes.h"
//Hier werden die Ergebnisse der Reaktionen auf Interrupts des DWM1000 festgehalten

//------Request

//------Request RX Timestamp

bool lookForRequestRxTs(uint8_t ID);

dwTime_t getReqRxTs(uint8_t ID);

void setReqRxTs(uint8_t ID, dwTime_t reqRxTsInput);


//------Request TX Timestamp

bool lookForRequestTxTs(uint8_t ID);

dwTime_t getReqTxTs(uint8_t ID);

void setReqTxTs(uint8_t ID, dwTime_t reqTxTsInput);


//------Immediate Answer

bool lookForImmediateAnswer(uint8_t ID);

st_message_t getImmediateAnswer(uint8_t ID);

void setImmediateAnswerRxTs(uint8_t ID, dwTime_t immAnsRxTs);


//------Immediate Answer RX Timestamp

bool lookForImmedatateAnswerRxTs(uint8_t ID);

dwTime_t getImmediateAnswerRxTs(uint8_t ID);



//------Immediate Answer TX Timestamp

bool lookForImmediatateAnswerTxTs(uint8_t ID);

dwTime_t getImmediateAnswerTxTs(uint8_t ID);

void setImmediateAnswerTxTs(uint8_t ID, dwTime_t immAnsTxTs);

//------Immediate ACK TX Timestamp
bool lookForAckAnswerTxTs(uint8_t ID) ;

void setAckAnswerTxTs(uint8_t ID, dwTime_t time) ;

dwTime_t getAckAnswerTxTs(uint8_t ID) ;

//------Processing Time

bool lookForProcessingTime(uint8_t ID);

dwTime_t getProcessingTime(uint8_t ID);

void setProcessingTime(uint8_t ID, dwTime_t procTime);

#endif

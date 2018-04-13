#ifndef ai_datatypesh
#define ai_datatypesh

#include "ai_config.h"
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden
#include "libdw1000Types.h"


//Rolle der Drohne
typedef enum {
	AI_NO_ROLE = 0,
	AI_MASTER,
	AI_SLAVE
} e_role_t;

//unsigned char my_ai_name = 0;
//e_role_t my_ai_role = AI_MASTER;

typedef struct {

	int curDisTab;												   //aktuelle Distance Tabel, Wert 0 oder 1, switch case in .c
	int timestamp[NR_OF_DRONES][NR_OF_DRONES];
	float distanceTable [MAX_HISTORY][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-2)][Drohnen Nummer][Distanz]
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distances_t;

typedef struct {
	double PAN_Identifier;	//pers�nlicher "Name" des DWM1000s im UWB-Bus
	//Baudrate, ...
}st_DWM_Config_t;

typedef struct { // struct für Koordinaten
	float x,y,z;
	float angle;
} st_coords_t;

typedef enum {
	IMMEDIATE_ANSWER_ACK = 6,
	DISTANCE_TABLE = 5,
	MASTER_STATE = 4,
	DISTANCE_REQUEST = 3,
	IMMEDIATE_ANSWER = 2,
	PROCESSING_TIME = 1,
	UNDEFINED = 0
} e_message_type_t;

typedef enum {
	FAILED_EVAL=0,
	RX_DONE,
	TX_DONE,
	RX_FAILED,
	RX_INT_TIMEOUT,
} e_interrupt_type_t;

/*
typedef struct {
	unsigned char senderID;						//Byte Name des Senders
	unsigned char targetID;						//Byte Name des Ziels
	e_message_type_t messageType;		//Art der Nachricht
	dwTime_t time;						//je nach Message time untersch. Bedeutung
	st_distances_t distanceTable;		//Tabelle mit Distanzen
} st_message_t;*/

typedef struct st_message_s {
	uint8_t destAddress;						// Byte 0-8:		Dest Address
	uint8_t sourceAddress;					// Byte 1-15:		Source Address
	e_message_type_t messageType;					// Byte 16:			Message Type
	union {/*
		struct {
			st_distances_t distanceTable;			// Byte 0-8:		Dest Address
			dwTime_t timestamp;						// Byte 0-8:		Dest Address
		} st_distance_broadcast_s;*/
		struct {
			dwTime_t receiveTimestamp;
			dwTime_t sendTimestamp;
		} distance_measurement_s;

	};
} __attribute__((packed)) st_message_t;

typedef enum {
	REQ_STATE_IDLE = 0,
	REQ_STATE_REQUESTED,
	REQ_STATE_IMMANSWERRECEIVE,
	REQ_STATE_IMMANSWERACK,
	REQ_STATE_CALCTROUND,
	REQ_STATE_CALCDIST,
} e_requestee_state_t;

typedef enum {
	TARGET_STATE_IDLE = 0,
	TARGET_STATE_DISTREQUESTED,
	TARGET_STATE_GETIMMANSWERTS,
	TARGET_STATE_READYPROCESSING,
} e_target_state_t;

typedef struct {
	float distance;							//distance measured in meters
	uint64_t lastRanginInAITicks;
	uint64_t rangingDuration;			//um zu prüfen, ob Ranging abgebrochen werden muss, da irgendetwas schief lief

	//requestee times
	dwTime_t requestTxTimestamp;
	dwTime_t immediateAnswerRxTimestamp;
	dwTime_t tRound;

	//target times
	dwTime_t processingTime;
	dwTime_t requestRxTimestamp;
	dwTime_t immediateAnswerTxTimestamp;


	e_requestee_state_t requesteeState;		//state if this is requestee
	e_target_state_t targetState;			//state if this is target




	//---requestee
	bool requestTransmitTimestampPending;//_/			//awaiting sent interrupt
	bool immediateAnswerPending;//_/					//if immediate answer hasnt been received jet
	bool processingTimePending;//				//awaiting processing time



	//---target
	bool distanceRequested;//_/						//distance requested by requestee (this is target)
	bool transmitProcessingTimePendingFlag;//_/



} st_rangingState_t;

#endif

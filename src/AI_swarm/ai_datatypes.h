#include "ai_config.h"
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden


//Rolle der Drohne
enum e_role_t {
	AI_ROLE_NO_ROLE = 0,
	AI_ROLE_MASTER,
	AI_ROLE_SLAVE
}; // e_role_t;

char my_ai_name = 0;
enum e_role_t my_ai_role = AI_ROLE_MASTER;

typedef struct st_distances_t {

	int curDisTab;												   //aktuelle Distance Tabel, Wert 0 oder 1, switch case in .c
	int timestamp[NR_OF_DRONES][NR_OF_DRONES];
	float distanceTable [MAX_HISTORY][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-2)][Drohnen Nummer][Distanz]
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
};

typedef struct st_DWM_Config_t {
	double PAN_Identifier;	//pers�nlicher "Name" des DWM1000s im UWB-Bus
	//Baudrate, ...
} st_DWM_Config_t;

enum e_message_type_t {
	DISTANCE_TABEL,
	MASTER_STATE,
	UNDEFINED = 0
} ;

enum e_interrupt_type_t {
	FAILED_EVAL=0,
	RX_DONE,
	TX_DONE
};

typedef struct  st_message_t {
	char senderID;						//Byte Name des Senders
	char targetID;						//Byte Name des Ziels
	enum e_message_type_t messageType;	//Art der Nachricht
	int rxSenderTimestamp;				//Receive Timestamp beim Sender
	int rxTargetTimestamp;				//Receive Timestamp beim Ziel
	int txSenderTimestamp;				//Transmit Timestamp beim Sender
	int txTargetTimestamp;				//Transmit Timestamp beim Ziel
	st_distances_t distanceTable;		//Tabelle mit Distanzen
};
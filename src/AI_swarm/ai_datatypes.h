#ifndef ai_datatypesh
#define ai_datatypesh

#include "ai_config.h"
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden


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
	DISTANCE_TABLE,
	MASTER_STATE,
	DISTANCE_REQUEST,
	IMMEDIATE_ANSWER,
	PROCESSING_TIME,
	UNDEFINED = 0
} e_message_type_t;

typedef enum {
	FAILED_EVAL=0,
	RX_DONE,
	TX_DONE,
	RX_FAILED,
	RX_INT_TIMEOUT,
} e_interrupt_type_t;

typedef long ai_time;	//time in picoseconds (10^-12s; Maximalwert ca. 9*10⁶s)

typedef struct {
	char senderID;						//Byte Name des Senders
	char targetID;						//Byte Name des Ziels
	e_message_type_t messageType;		//Art der Nachricht
	ai_time time;						//je nach Message time untersch. Bedeutung
	st_distances_t distanceTable;		//Tabelle mit Distanzen
} st_message_t;



#endif

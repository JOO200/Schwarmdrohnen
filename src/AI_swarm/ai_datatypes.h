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

	int curDisTab;										//aktuelle Distance Tabel, Wert 0 oder 1, switch case in .c
	int timestamp[NR_OF_DRONES][NR_OF_DRONES];
	float distanceTable [MAX_HISTORY][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-2)][Drohnen Nummer][Distanz]
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distaces_t;

typedef struct st_DWM_Config_t {
	double PAN_Identifier;	//pers�nlicher "Name" des DWM1000s im UWB-Bus
	//Baudrate, ...
} st_DWM_Config_t;

enum e_message_type_t {
	DISTANCE_TABEL,
	MASTER_STATE,
	UNDEFINED = 0

} ;


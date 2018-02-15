#include "ai_config.h"
#include "ai_locodeck.h"
#include "stddef.h"	//für z.B. NULL muss dies includiert werden


/*//Struct, welches Abstandsdaten mit Bezug auf eine gennannte Drohne enthält
typedef struct {				
	int sender;					//Drohne auf die sich die Distanzen beziehen
	float dist_to_master;		//Entfernung zu Masterdrohne
	float dist_to [NR_OF_DRONES];
}st_distances_t;				*/

//Rolle der Drohne
typedef enum e_role_t {
	MASTER, 
	SLAVE,
	UNDEFINED = 0
} e_role_t;

e_role_t my_ai_role = UNDEFINED;

//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen
typedef struct st_distances_t {
	float distanceTable[MAX_HISTORY][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-2)][Drohnen Nummer][Distanz]
	int curDisTab;										//aktuelle Distance Tabel, Wert 0 oder 1, switch case in .c
	time_t timestamp[NR_OF_DRONES][NR_OF_DRONES];
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distaces_t;

//config des DWM-Boards
typedef struct st_DWM_Config_t {
	double PAN_Identifier;	//persönlicher "Name" des DWM1000s im UWB-Bus
	//Baudrate, ...
} st_DWM_Config_t;

//enum um zu spezifizieren, welche Art von Nachricht aus dem DWM1000 ausgelesen wurde
typedef enum e_message_type_t {
	DISTANCE_TABEL,
	MASTER_STATE,
	UNDEFINED = 0

} e_message_type_t;

//----------------------------------- DWM1000 Interface -------------------------------------



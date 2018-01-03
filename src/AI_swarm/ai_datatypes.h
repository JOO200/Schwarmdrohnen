#include "ai_config.h"
#include "ai_locodeck.h"
#include "stddef.h"	//für z.B. NULL muss dies includiert werden


//Struct, welches Abstandsdaten mit Bezug auf eine gennannte Drohne enthält
typedef struct {				
	int sender;					//Drohne auf die sich die Distanzen beziehen
	float dist_to_master;		//Entfernung zu Masterdrohne
	float dist_to [NR_OF_DRONES];
}st_distances_t;				

//Rolle der Drohne
typedef enum e_role_t {
	MASTER,
	SLAVE,
	UNDEFINED = 0
} e_role_t;

e_role_t my_ai_role = UNDEFINED;


//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen
typedef struct st_distances_t {
	float distanceTable[NR_OF_DRONES][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-4)][Drohnen Nummer][Distanz]
	int curdisTab;												//aktuelle Distance Tabel, vorherigen 3 = history, switch case in .c
	float history = [NR_OF_DRONES];									//distanceTabel1=history[1]					
	time_t timestamp[NR_OF_DRONES][NR_OF_DRONES];
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distaces_t;

//----------------------------------- DWM1000 Interface -------------------------------------



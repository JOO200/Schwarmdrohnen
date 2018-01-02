#include "ai_config.h"
#include "stddef.h"	//für z.B. NULL muss dies includiert werden


//Struct, welches Abstandsdaten mit Bezug auf eine gennannte Drohne enthält
typedef struct {				
	int sender;					//Drohne auf die sich die Distanzen beziehen
	float dist_to_master;		//Entfernung zu Masterdrohne
	float dist_to[MAX_DRONES];
}st_distances_t;				

//Rolle der Drohne
typedef enum e_role_t {
	MASTER,
	SLAVE,
	UNDEFINED = 0
} e_role_t;

e_role_t my_ai_role = UNDEFINED;


//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen

//----------------------------------- DWM1000 Interface -------------------------------------



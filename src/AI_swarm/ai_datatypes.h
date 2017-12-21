#include "ai_config.h"


//Struct, welches Abstandsdaten mit Bezug auf eine gennannte Drohne enthält
typedef struct {				
	int sender;					//Drohne auf die sich die Distanzen beziehen
	float dist_to_master;		//Entfernung zu Masterdrohne
	float dist_to[MAX_DRONES];
}st_distances_t;				

//Rolle der Drohne
typedef enum {
	MASTER,
	SLAVE,
	UNDEFINED = 0
} e_role_t;


//----------------------------------- DWM1000 Interface -------------------------------------
//test ob i recht hab



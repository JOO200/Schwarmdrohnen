/* Managen der Distanz Tabellen

typedef struct st_distances_t {
	float distanceTable[NR_OF_DRONES][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-4)][Drohnen Nummer/Adresse][Distanz]
	float curdisTab;												//aktuelle Distance Tabel, vorherigen 3 = history, switch case in .c
	float history = [NR_OF_DRONES][NR_OF_DRONES][NR_OF_DRONES];		//distanceTabel1=history[1]					
	time_t timestamp[NR_OF_DRONES][NR_OF_DRONES];
	//history ...
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distaces_t;

include für DEBUG_PRINT
*/

#include "ai_datatypes.h"

switch (curdisTab)
{
case 0:
	history[0] = distanceTable[0];
	break;
case 1:

	break;
case 2:

	break;
case 3:

	break;
default:
	DEBUG_PRINT("Failed to configure Distance Table!\r\n");
	break;
}
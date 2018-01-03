/* Managen und einlesen der Distanz Tabellen

Es gibt 2 Distanz-Tabelle, Seite 0 und Seite 1. curdisTab (Current Distance Table) gibt die Seitenzahl an.
Beim einlesen der Distanzen muss curdisTab nach jedem Zyklus negiert werden.
Die jeweils nicht aktuelle Seite ist damit automatisch die History Page.

Distanzberechnung über eigenen Thread?

include für DEBUG_PRINT

Momentan noch Nonsense, muss überdacht werden
*/



#include "ai_datatypes.h"

//Seite der aktuellen Distanz-Tabelle 

while (//array nicht voll)
	{
		distanceTable[curdisTab][Drone_NR][get_distance()];
	}

if (curdisTab == 0)
{
	curdisTab = 1;
}
else if (curdisTab == 1)
{
	curdisTab = 0;
}

/*switch (curdisTab)
{
case 0:
	distanceTable
	break;
case 1:

	break;
default:
	DEBUG_PRINT("Failed to configure Distance Table!\r\n");
	break;
}*/
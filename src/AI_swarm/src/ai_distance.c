/* Managen und einlesen der Distanz Tabellen

Es gibt 2 Distanz-Tabelle, Seite 0 und Seite 1. curDisTab (Current Distance Table) gibt die Seitenzahl an.
Beim einlesen der Distanzen muss curDisTab nach jedem Zyklus negiert werden.
Die jeweils nicht aktuelle Seite ist damit automatisch die History Page.

Distanzberechnung über eigenen Thread?
	-- workerSchedule habe ich mich schon drum gekümmert - Nicolai

include für DEBUG_PRINT

Momentan noch Nonsense, muss überdacht werden
*/



#include "ai_datatypes.h"

//Seite der aktuellen Distanz-Tabelle 

while (//array nicht voll)
	{
		//Anpassung der "Adress-Aufrufe". Wie können die einzelnen Drohnen als Ganzzahlen von 0-3 angesprochen werden?
		//CurrentDrone_NR=0;
		//VergleichsDrone_NR=0;

		distanceTable[curDisTab][CurrentDrone_NR][VergleichsDrone_NR] =get_distance();

//CurrentDrone_NR, VergleichsDrone_NR hochzählen
	}

//Umschalten der Tabllen Seiten nach jedem Dateneinlesen
if (curDisTab == 0)
{
	curDisTab = 1;
}
else if (curDisTab == 1)
{
	curDisTab = 0;
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
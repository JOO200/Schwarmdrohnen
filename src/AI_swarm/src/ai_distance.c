/* Managen und einlesen der Distanz Tabellen

Es gibt 2 Distanz-Tabelle, Seite 0 und Seite 1. curdisTab (Current Distance Table) gibt die Seitenzahl an.
Beim einlesen der Distanzen muss curdisTab nach jedem Zyklus negiert werden.
Die jeweils nicht aktuelle Seite ist damit automatisch die History Page.

Distanzberechnung �ber eigenen Thread?

include f�r DEBUG_PRINT

Momentan noch Nonsense, muss �berdacht werden
*/



#include "ai_datatypes.h"

//Seite der aktuellen Distanz-Tabelle 

while (//array nicht voll)
	{
		//Anpassung der "Adress-Aufrufe". Wie k�nnen die einzelnen Drohnen als Ganzzahlen von 0-3 angesprochen werden?
		//CurrentDrone_NR=0;
		//VergleichsDrone_NR=0;

		distanceTable[curdisTab][CurrentDrone_NR][VergleichsDrone_NR] =get_distance();

//CurrentDrone_NR, VergleichsDrone_NR hochz�hlen
	}

//Umschalten der Tabllen Seiten nach jedem Dateneinlesen
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
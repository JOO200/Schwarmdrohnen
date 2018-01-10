/* Managen und einlesen der Distanz Tabellen

Es gibt 2 Distanz-Tabelle, Seite 0 und Seite 1. curDisTab (Current Distance Table) gibt die Seitenzahl an.
Beim einlesen der Distanzen muss curDisTab nach jedem Zyklus negiert werden.
Die jeweils nicht aktuelle Seite ist damit automatisch die History Page.

Distanzberechnung über eigenen Thread?
	-- workerSchedule habe ich mich schon drum gekümmert - Nicolai

include für DEBUG_PRINT

Ablauf:
Master -> call UpdateDistanceTabel() in ai_task.c
{ 
	Master startet distance update in Slaves
	Master updatet seine eigenen Distanzen
	Master fordert alle Slave Tabels an
	{ 
		Slaves schicken erste wenn fertig berechnet (z.B. Semaphore)
	} 
	Master verteilt vollständige Tabelle
}

*/

#include "ai_datatypes.h"
#include "ai_config.h"
#include "ai_distance.h"


void UpdateDistanceTable()
{
	//Anpassung der "Adress-Aufrufe". Wie können die einzelnen Drohnen als Ganzzahlen von 0-3 angesprochen werden?
	//CurrentDrone_NR=0;
	//VergleichsDrone_NR=0;
	int CurrentDrone_NR = UWB_NAME;
	for (int VergleichsDrone_NR = 0; VergleichsDrone_NR <5; VergleichsDrone_NR++)
	{
		distanceTable[curDisTab][CurrentDrone_NR][VergleichsDrone_NR] = calculate_distance();
	}

	//Tabellenschnipsel an MASTER senden
	//MASTER versendet ganze Tabelle

	if (my_ai_role == MASTER)
	{
		get_distances(); //von allen Drohnen die Distanztabellen bekommen
		distribute_distances(); //versendet vollständige Distanztabelle
	}
	
	//Umschalten der Tabllen Seiten nach jedem Dateneinlesen
	if (MAX_HISTORY > 0)
	{
		curDisTab += 1;
		if (curDisTab > MAX_HISTORY)
		{
			curDistTab = 0;
		}
	}
}

//Tabelle mit Abständen füllen
float calculate_distance()
{

}

//von allen Drohnen die Distanztabellen bekommen
float get_distances() 
{

}

//versendet vollständige Distanztabelle
float distribute_distances() 
{

}
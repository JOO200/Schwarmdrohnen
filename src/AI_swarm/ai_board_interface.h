//Includes...
#include "ai_datatypes.h"
//...

//-----Funktionale Methoden

//Senden von Abstandstabelle
//@return true if ack
//
bool aiSendDistanceTable(st_distances_t distances);

//Master kann diese Methode nutzen um Distanzaustausch einzuleiten
//Distanzen werden in Array von st_distances_t geschrieben
//@return true, falls von allen Antwort
bool aiRequestDistances(st_distances_t *  slaveDistances1);

//1. UWB Verbindung herstellen - timeout-Zeit, Fernbedienungsverbindung erstmal unmöglich
//2. Drohne mit niedrigster ID wird Master, alle anderen Slave
//3. Master wartet auf Fernbedienungsverbindung
//
void aiInit();






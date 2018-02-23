/*
 * In dieser Methode werden die komplette Motorberechnung ausgeführt.
 */
#include "ai_datatypes.h"
#include "ai_config.h"
#include "ai_triangulation.c"
#include "ai_distance.c"
#include "ai_pid.c"
#include "power_distribution_stock.c"

static st_coords_t ** history = st_coords_t[NR_OF_DRONES];
static currHistory;

/**
 *  Diese Methode aktualisiert die Abstandstabelle, trianguliert die Positionen und berechnet die Motoransteuerung.
 *  Es ist zu überprüfen, in wie fern der stabilizerTask (Task-Name: STABILIZER) mit diesem Task in die Quere kommt.
 *  Beide Tasks greifen auf die Motoren zu. Sollte es zu Problemen kommen, so ist vor dem Benutzen dieser Methode der
 *  Task "STABILIZER" zu deaktivieren oder zu pausieren, allerdings nur, solange es keine Master-Drohne ist.
 *
 *  Zu beachten gilt: ideal muss Arrays mit der Größe von NR_OF_DRONES sein.
 *
 *  @param distances struct mit aktuellen Distanzen
 *  @param ideal Ideale Koordinaten (Koordinaten der Flugformation). Array mit der Größe NR_OF_DRONES
 *  @param master_control aktuelles Steuerverhalten der Masterdrohne.
 *
 *  @return null - es wird nichts zurückgegeben
 */
void makeMotors(st_distances_t * distances, st_coords_t * ideal, control_s * master_control) {
    // st_coords_t * real_coords = st_coords_t[NR_OF_DRONES];

    st_coords_t * real_coords = (st_coords*)malloc(NR_OF_DRONES*sizeof(st_coords_t));
    control_s * control = (control_s*)malloc(sizeof(control_s));
    if(real_coords == NULL || control == NULL) {
        return;
    }
    UpdateDistanceTable(distances);					// Abastandstabelle aktuell halten. -> das wird evtl. ein anderer Task machen.
    triangulate(distances, ideal, real_coords);		// triangulieren der aktuelen Position. (benötigt history?)

    pid_regler(control, ideal, real_coords, master_control); // PID-Regler (benötigt history von den realen Koordinaten?)

    powerDistribution(control); // Motoren angesteuert


    backupRealCoords(real_coords); // in einen Speicher schreiben

    //Speicher leeren
    free(real_coords);
    free(control);
}

void backupRealCoords(real_coords) {
	// TODO: Backup für PID-Regler erstellen
}

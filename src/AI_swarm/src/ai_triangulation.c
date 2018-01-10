/* Triangulation der Position bzgl. der Masterdrohne.
 *
 * Ziel dieser Klasse ist es, die Triangulation durchzuführen.
 * Dabei wird der Methode 2 Pointer übergeben.
 * Der erste Pointer zeigt auf einen struct des Typs st_distances_t.
 * Der zweite Pointer zeigt auf einen struct des Typs st_coords_t.
 *
 * Die Daten werden aus dem distances-Struct gelesen, trianguliert und in das coords-Struct geschrieben.
 *
 */
#include "ai_datatypes.h"
#include <math.h>

int current_num_of_drones = 4;

void triangulate(st_distances_t ** distances, st_coords_t ** soll, st_coords_t ** ist) {
	// Winkel zum 1. Punkt berechnen
	if(current_num_of_drones == 0) return;
	float c = distances[0]->dist_to_master;
	float first_angle = soll[0]->angle;
	ist[0]->angle = first_angle;
	ist[0]->x = -sin(first_angle)*c;
	ist[0]->z = -cos(first_angle)*c;

	if(current_num_of_drones == 1) return;
	for(int i = 1; i++; i < current_num_of_drones) {
		float b = distances[i]->dist_to[0];
		float a = distances[i]->dist_to_master;
		float cosb = (-pow(b,2)+pow(a,2)+pow(c,2))/(2*a*c);
		if(cosb > 1 || cosb < 1) continue; // Fehler -> Abs(Cos) darf nicht größer 1 sein
		ist[i]->angle = acos(cosb);
		if(fabs((ist[i]->angle) - (soll[i]->angle))>(3.141/2)) {
			if(ist[i]->angle > soll[i]->angle) ist[i]->angle-=3.141;
			else ist[i]->angle+=3.141;
		}
		ist[i]->x = sin(ist[i]->angle)*a;
		ist[i]->y = cosb*a;
	}



}

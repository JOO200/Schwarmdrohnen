/*
 * In dieser Klasse wird über einen PID-Regler die Ansteuerung der Motoren bewirkt.
 * Die Werte werden in einem Struct vom Typ "control_s" gespeichert. Dieses Struct ist von Bitcraze schon vorhanden.
 */

void pid_regler(control_s * control, st_coords_t ** ideal, st_coords_t ** real, control_s * master_control) {
    //TODO: PID Regler hinzufügen. Dabei die Vergangenheit der Drohnen "prüfen".

    control_s = master_control;     // vorerst nutzen wir keinen Regler sondern übernehmen die Signale alle 1 zu 1.

    //Eventuell kann hier auch der PID-Regler von Crazyflie eingesetzt werden: controller_pid.c
}
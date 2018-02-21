#include "ai_config.h"
//#include "ai_locodeck.h"
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden

//Rolle der Drohne
typedef enum e_role_t;
//typedef enum e_role_t State;

//static enum e_role_t my_ai_role = AI_ROLE_NO_ROLE;

//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen
typedef struct st_distances_t;

//config des DWM-Boards
typedef struct st_DWM_Config_t;

//enum um zu spezifizieren, welche Art von Nachricht aus dem DWM1000 ausgelesen wurde
 typedef enum  e_message_type_t;

//----------------------------------- DWM1000 Interface -------------------------------------



#include "ai_config.h"
<<<<<<< HEAD
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden


/*//Struct, welches Abstandsdaten mit Bezug auf eine gennannte Drohne enth�lt
typedef struct {				
	int sender;					//Drohne auf die sich die Distanzen beziehen
	float dist_to_master;		//Entfernung zu Masterdrohne
	float dist_to [NR_OF_DRONES];
}st_distances_t;				*/
=======
//#include "ai_locodeck.h"
#include "stddef.h"	//f�r z.B. NULL muss dies includiert werden
>>>>>>> 87396e1998d46bfab222aa232f3e62b443bc2a99

//Rolle der Drohne
typedef enum e_role_t;
//typedef enum e_role_t State;

<<<<<<< HEAD
e_role_t my_ai_role = UNDEFINED;
char my_ai_name = 0;

//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen
typedef struct st_distances_t {
	float distanceTable[NR_OF_DRONES][NR_OF_DRONES]; //[Drohnen Nummer][Distanz]
	int curDisTab;										//aktuelle DistanceTabel, Wert 0 oder 1, switch case in .c
	time_t timestamp[NR_OF_DRONES][NR_OF_DRONES];
	//history ... Entscheidung: keine History
	//Semaphoren? Staus Vaiable (running, waiting, ready)?
	//Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distaces_t;

//config des DWM-Boards
typedef struct st_DWM_Config_t {
	double PAN_Identifier;	//pers�nlicher "Name" des DWM1000s im UWB-Bus
	//Baudrate, ...
	
} st_DWM_Config_t_default = { PAN_Identifier = my_ai_name, Bitrate = };
=======
//static enum e_role_t my_ai_role = AI_ROLE_NO_ROLE;

//Vorschlag: mehrdimensionales quadratisches Array mit Distanzen aller Drohnen, Synchronisierung der Haupt- und Nebendiagonalen
typedef struct st_distances_t;

//config des DWM-Boards
typedef struct st_DWM_Config_t;
>>>>>>> 87396e1998d46bfab222aa232f3e62b443bc2a99

//enum um zu spezifizieren, welche Art von Nachricht aus dem DWM1000 ausgelesen wurde
 typedef enum  e_message_type_t;

//----------------------------------- DWM1000 Interface -------------------------------------
//Struct welches DMW1000 darstellt
typedef struct dwDevice_s {
	struct dwOps_s *ops;
	void *userdata;

	/* State */
	uint8_t sysctrl[LEN_SYS_CTRL];
	uint8_t deviceMode;
	uint8_t networkAndAddress[LEN_PANADR];
	uint8_t syscfg[LEN_SYS_CFG];
	uint8_t sysmask[LEN_SYS_MASK];
	uint8_t chanctrl[LEN_CHAN_CTRL];
	uint8_t sysstatus[LEN_SYS_STATUS];
	uint8_t txfctrl[LEN_TX_FCTRL];

	uint8_t extendedFrameLength;
	uint8_t pacSize;
	uint8_t pulseFrequency;
	uint8_t dataRate;
	uint8_t preambleLength;
	uint8_t preambleCode;
	uint8_t channel;
	/*bool smartPower;
	bool frameCheck;
	bool permanentReceive;
	bool wait4resp;*/

	//dwTime_t antennaDelay;

	// Callback handles
	dwHandler_t handleSent;
	dwHandler_t handleReceived;
	dwHandler_t handleReceiveTimeout;
	dwHandler_t handleReceiveFailed;
} dwDevice_t;


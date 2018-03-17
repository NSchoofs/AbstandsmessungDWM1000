#pragma once
#include "ai_config.h"
#include <DW1000Time.h>
#include <DW1000Ranging.h>
#include <DW1000Mac.h>
#include <DW1000Device.h>
#include <DW1000Constants.h>
#include <DW1000CompileOptions.h>
#include <DW1000.h>

typedef enum {
	AI_NO_ROLE = 0,
	AI_MASTER,
	AI_SLAVE
} e_role_t;

//unsigned char my_ai_name = 0;
//e_role_t my_ai_role = AI_MASTER;

typedef struct {

	int curDisTab;												   //aktuelle Distance Tabel, Wert 0 oder 1, switch case in .c
	int timestamp[NR_OF_DRONES][NR_OF_DRONES];
	float distanceTable[MAX_HISTORY][NR_OF_DRONES][NR_OF_DRONES]; //[Tabellen Nummer (1-2)][Drohnen Nummer][Distanz]
																  //history ...
																  //Semaphoren? Staus Vaiable (running, waiting, ready)?
																  //Fkt zum Vergleich der Haupt- Nebendiagonale
} st_distances_t;

typedef struct {
	double PAN_Identifier;	//pers�nlicher "Name" des DWM1000s im UWB-Bus
							//Baudrate, ...
}st_DWM_Config_t;

typedef struct { // struct für Koordinaten
	float x, y, z;
	float angle;
} st_coords_t;

typedef enum {
	DISTANCE_TABLE,
	MASTER_STATE,
	DISTANCE_REQUEST,
	IMMEDIATE_ANSWER,
	PROCESSING_TIME,
	UNDEFINED = 0
} e_message_type_t;

typedef enum {
	FAILED_EVAL = 0,
	RX_DONE,
	TX_DONE,
	RX_FAILED,
	RX_INT_TIMEOUT,
} e_interrupt_type_t;

typedef struct {
	unsigned char senderID;						//Byte Name des Senders
	unsigned char targetID;						//Byte Name des Ziels
	e_message_type_t messageType;		//Art der Nachricht
	DW1000Time time;						//je nach Message time untersch. Bedeutung
	st_distances_t distanceTable;		//Tabelle mit Distanzen
} st_message_t;

typedef struct {
	//---requestee
	bool requestTransmitTimestampPending;//_/			//awaiting sent interrupt
	bool immediateAnswerPending;//_/					//if immediate answer hasnt been received jet
	bool processingTimePending;//				//awaiting processing time

	DW1000Time lastRanging;							//time of last ranging
	float distance;									//distance measured in meters

													//requestee times
	DW1000Time requestTxTimestamp;
	DW1000Time immediateAnswerRxTimestamp;
	DW1000Time tRound;

	//---target
	bool distanceRequested;//_/						//distance requested by requestee (this is target)
	bool transmitProcessingTimePendingFlag;//_/

										   //target timestamps
	DW1000Time requestRxTimestamp;
	DW1000Time immediateAnswerTxTimestamp;

} st_rangingState_t;


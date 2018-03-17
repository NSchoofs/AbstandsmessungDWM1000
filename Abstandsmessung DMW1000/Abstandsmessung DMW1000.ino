#include <dummy.h>
#include <SPI.h>
#include <require_cpp11.h>
#include <DW1000Time.h>
#include <DW1000Ranging.h>
#include <DW1000Mac.h>
#include <DW1000Device.h>
#include <DW1000Constants.h>
#include <DW1000CompileOptions.h>
#include <DW1000.h>
#include <deprecated.h>
#include "ai_task.h"
#include "dwm1000Driver.h"
#include "ai_datatypes.h"


//initialisieren vom Externen Interrupt (Anleitung von: https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/):
const byte interruptPin = 25;
volatile int interruptCounter;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void setup()
{
	Serial.begin(115200);
	pinMode(interruptPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);

	if (ai_init == 0) {
		initAi_Swarm();
		ai_init = 1;
	}
	Serial.println("Setup done...");
}


bool ai_init = 0;
st_message_t testMessage;
bool DMW1000_IRQ_Flag = 0;

//Ranging Flags, state and Distance:
st_rangingState_t rangingState[NR_OF_DRONES];

e_message_type_t lastMessageType;
unsigned char lastMessageTarget;


void receiveHandler() {
	Serial.println("Message received:");

	st_message_t message;
	dwm1000_ReceiveData(&message);
	Serial.println("Message target: " + message.targetID);
	Serial.println("Message sender: " + message.senderID);

	if (message.targetID != AI_NAME)
		return;

	switch (message.messageType)
	{
	case DISTANCE_TABLE:
		//1. aktuelle Distance Tabelle holen
		//2. an Absender zurück schicken
		break;
	case MASTER_STATE:
		//Status des Masters aktualisieren
		break;
	case DISTANCE_REQUEST:
		rangingState[message.senderID].distanceRequested = true;
		//1. Immediate Answer raussenden
		st_message_t immediateAnswer;
		immediateAnswer.senderID = AI_NAME;
		immediateAnswer.targetID = immediateAnswer.senderID;
		immediateAnswer.messageType = IMMEDIATE_ANSWER;
		dwm1000_SendData(&immediateAnswer);
		//2. danach Processing Time nachsenden (pending flag setzen)
		rangingState[message.senderID].transmitProcessingTimePendingFlag = true;
		break;
	case IMMEDIATE_ANSWER:
		//Tround berechnen
		if (!rangingState[message.senderID].immediateAnswerPending)
			break;
		rangingState[message.senderID].immediateAnswerRxTimestamp = dwm1000_getRxTimestamp();
		rangingState[message.senderID].tRound = rangingState[message.senderID].immediateAnswerRxTimestamp - rangingState[message.senderID].requestTxTimestamp;
		rangingState[message.senderID].immediateAnswerPending = false;

		rangingState[message.senderID].processingTimePending = true;
		break;
	case PROCESSING_TIME:
		rangingState[message.senderID].processingTimePending = false;
		//Distanz berechnen und eintragen
		//Rangin-Struct leeren

		//Zeitberechnung in ai_lpsTwrTag Zeile 172-202
		break;
	default:
		break;
	}
}

void transmitDoneHandler() {
	if (rangingState[lastMessageTarget].transmitProcessingTimePendingFlag && lastMessageType == IMMEDIATE_ANSWER) {
		dwm1000_sendProcessingTime(lastMessageTarget);
		rangingState[lastMessageTarget].transmitProcessingTimePendingFlag = false;
	}
	else if (rangingState[lastMessageTarget].requestTransmitTimestampPending && lastMessageType == DISTANCE_REQUEST) {
		rangingState[lastMessageTarget].requestTxTimestamp = dwm1000_getTxTimestamp();
		rangingState[lastMessageTarget].requestTransmitTimestampPending = false;
	}
	else
		return;
}

//eventuell muessen args als void *
void startRanging(unsigned char targetID) {
	//requestMessage senden
	dwm1000_requestDistance(targetID);

	//flags und vars für Momentanzustand setzen
	rangingState[targetID].requestTransmitTimestampPending = true;
	rangingState[targetID].immediateAnswerPending = true;

	lastMessageType = DISTANCE_REQUEST;
	lastMessageTarget = targetID;
}


bool initAi_Swarm() {
	//UWB_Deck fuer Josy und Janik ((((neuerdings))) auch Nico)
	//hier euer/unser init-shizzlel
	setup_dwm1000_communication();		//HW-Setup


										//...
	return true;
}

void loop()
{
	float distance = 0;
	for (unsigned int i = 0; i > 60000; i++) {
		distance = (float)distance + (float)10 / (float)100;
		//ai_showDistance(distance);
		vTaskDelay(5000);
	}
	//... repetetives

	if (interruptCounter > 0) {		//"ISR"
		portENTER_CRITICAL(&mux);
		interruptCounter--;
		portEXIT_CRITICAL(&mux);

		e_interrupt_type_t interruptType = dwm1000_EvalInterrupt();

		switch (interruptType) {
		case RX_DONE:
			receiveHandler();
			break;
		case TX_DONE:
			transmitDoneHandler();
			break;
		default:
			//Unhandled messaging errors
			break;
		}
	}

	for (unsigned char i = 0; i < NR_OF_DRONES; i++) {
		if (i == AI_NAME) {	//nicht zu mir selbst rangen
			continue;
		}


		if (rangingState[i].distanceRequested == true)
			dwm1000_immediateDistanceAnswer(i);

		//dwTime_t timeSinceRanging = time.now - lastRanging[i];		//Zeit bestimmen, seid der Entfernung zu dieser Drohne das letzte mal bestimmt wurde
		//if (timeSinceRanging >= 1/RANGING_FREQUENCY){
		//	startRanging(i);											//falls diese über Schwellenwert --> neu Rangen
	}
	if (!PASSIVE_MODE && !rangingState[i].distanceRequested & !rangingState[i].requestTransmitTimestampPending & !rangingState[i].processingTimePending) {
		startRanging(i);
	}
}

void IRAM_ATTR handleInterrupt() {
	portENTER_CRITICAL_ISR(&mux);
	interruptCounter++;
	portEXIT_CRITICAL_ISR(&mux);
}



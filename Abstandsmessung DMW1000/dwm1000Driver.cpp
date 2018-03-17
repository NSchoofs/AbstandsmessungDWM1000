// 
// 
// 

#include "dwm1000Driver.h"
#include <DW1000Time.h>
#include <DW1000Ranging.h>
#include <DW1000Mac.h>
#include <DW1000Device.h>
#include <DW1000Constants.h>
#include <DW1000CompileOptions.h>
#include <DW1000.h>

/*
ai_dwm1000_driver.c

Diese Datei definiert die Funktionen, welche benoetigt werden, um den DWM1000 zu steuern (Daten Senden, Ranging, ...).
Zusaetzlich werden die wichtigen Registerbkonfiguriert.

Sollten keine Funktionsaenderungen des DWM1000 gewuenscht sein, sollte dieser Driver am Besten NICHT geaendert werden!
*/


#include "../ai_datatypes.h"
#include "../ai_task.h"


//Hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

static DW1000Class DW1000;
//static DW1000Class *dwm = &dwm_device;


/************ Low level ops for libdw **********/
static void spiRead(DW1000Class* dev, const void *header, size_t headerLength,
	void* data, size_t dataLength)
{
	spiBeginTransaction(spiSpeed);
	digitalWrite(CS_PIN, LOW);
	memcpy(spiTxBuffer, header, headerLength);
	memset(spiTxBuffer + headerLength, 0, dataLength);
	spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
	memcpy(data, spiRxBuffer + headerLength, dataLength);
	digitalWrite(CS_PIN, HIGH);
	spiEndTransaction();
}
static void spiWrite(DW1000Class* dev, const void *header, size_t headerLength,
	const void* data, size_t dataLength)
{
	spiBeginTransaction(spiSpeed);
	digitalWrite(CS_PIN, LOW);
	memcpy(spiTxBuffer, header, headerLength);
	memcpy(spiTxBuffer + headerLength, data, dataLength);
	spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
	digitalWrite(CS_PIN, HIGH);
	spiEndTransaction();
}
static void spiSetSpeed(DW1000Class* dev, dwSpiSpeed_t speed)
{
	if (speed == dwSpiSpeedLow)
	{
		spiSpeed = SPI_BAUDRATE_2MHZ;
	}
	else if (speed == dwSpiSpeedHigh)
	{
		spiSpeed = SPI_BAUDRATE_21MHZ;
	}
}
static void delayms(DW1000Class* dev, unsigned int delay)
{
	vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
	.spiRead = spiRead,
	.spiWrite = spiWrite,
	.spiSetSpeed = spiSetSpeed,
	.delayms = delayms,
};


//Functions for interrupt handling
e_interrupt_type_t lastInterrupt;
static void dwm1000_transmitDoneHandler() {
	lastInterrupt = TX_DONE;
}

static void dwm1000_receiveHandler() {
	lastInterrupt = RX_DONE;
}

static void dwm1000_receiveFailedHandler() {
	lastInterrupt = RX_FAILED;
}

static void dwm1000_receiveTimeoutHandler() {
	lastInterrupt = RX_INT_TIMEOUT;
}


//Inhalt von locodec.c init (ab Z.312) inspiriert (und angepasst)
bool setup_dwm1000_communication() {





// init with interrupt line and optionally with reset line
DW1000.begin(irq_pin[, rst_pin]);
// select a specific chip via a chip select line
DW1000.select(cs_pin);
// or use DW1000.reselect(cs_pin) to switch to previously selected chip
//...
		// open a device configuration sessions
	DW1000.newConfiguration();
	// configure specific aspects and/or choose defaults
	DW1000.setDefaults();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(AI_NAME);
	// modes that define data rate, frequency, etc. (see API docs)
	DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY);
// ... and other stuff - finally upload to the module.
DW1000.commitConfiguration();
//...
		// set some interrupt callback routines
	DW1000.attachSentHandler(dwm1000_transmitDoneHandler);
	DW1000.attachReceivedHandler(dwm1000_receiveHandler);
	DW1000.attachReceiveFailedHandler(dwm1000_receiveFailedHandler);
	DW1000.attachReceiveTimeoutHandler(dwm1000_receiveTimeoutHandler);
	//...


	// similar for a receiving session, like so ...
	DW1000.newReceive();
DW1000.setDefaults();
// so we don't need to restart the receiver manually each time
DW1000.receivePermanently(true);
// ... and other stuff - finally start awaiting messages
DW1000.startReceive();
	return 1;
}


bool dwm1000_SendData(st_message_t *message) {
	DW1000.newTransmit();
	DW1000.setDefaults();
	int msgSize = sizeof(st_message_t);
	DW1000.setData(DW1000, (uint8_t*)(void*)message, msgSize);

	DW1000.startTransmit();

	//turn receive back on?
	DW1000.newReceive();
	DW1000.setDefaults();
	DW1000.receivePermanently(true);
	DW1000.startReceive();

	return 1;
}

e_message_type_t dwm1000_ReceiveData(st_message_t &data) {
	int dataLength = DW1000.getDataLength();

	if (dataLength == 0) {
		return 	UNDEFINED;		//if Fall fuer leere Empfangsdaten
	}
	if (dataLength != sizeof(st_message_t)) {
		return UNDEFINED;
	}

	memset(&data, 0, dataLength);  //packet mit Nullen ueberschreiben

	DW1000.getData((uint8_t*)&data, dataLength);	//get Packet und befuellen

	return data.messageType;

}


//---------------------- RANGING ----------------------

/*
Sendet Distanzrequest an spezifiziertes Target

// Beschreibung des gesammten Ranginvorgangs:
//1. Nachrichten an Partner schicken (requester)
Flag setzen: requestTransmitTimestampPending

//2. Partner antwortet  direkt (target)
Flag setzen: immediateAnswerTransmitTimestampPending

//3. Master empfägt target-Nachricht - berechnet T_round (Zeit die die Nachricht hin und zurück gebraucht hat)
Flag setzen: processingTimePending

//4. target berechnet seine Bearbeitungszeit (immediateAnswerTranceiveTimestamp - requestReceiveTimestamp)

//5. (Gestoppte Zeit - (Bearbeitungstimestamp))/2 * Lichtgeschw = Abstand

// Danach weiß der requester den Abstand
*/
void dwm1000_requestDistance(char targetID) {
	st_message_t requestMessage;
	requestMessage.senderID = AI_NAME;
	requestMessage.targetID = targetID;
	requestMessage.messageType = DISTANCE_REQUEST;

	dwm1000_SendData(&requestMessage);
}
/*
//1. Funktion aktiviert, nach Distance request Eingang
//2. Antworten, damit requester Zeit stopppen kann

//Zusatz: Zeit zwischen Receive Timestamp und Transmit Timestamp an requester schicken, damit von gestoppter Zeit abgezogen werden kann
//Receive Timestamp - Transmit Timestamp
*/
void dwm1000_immediateDistanceAnswer(char id_requester) {
	st_message_t immediateAnswer;
	immediateAnswer.senderID = AI_NAME;
	immediateAnswer.targetID = id_requester;
	immediateAnswer.messageType = IMMEDIATE_ANSWER;

	dwm1000_SendData(&immediateAnswer);
}

void dwm1000_sendProcessingTime(char id_requester, DW1000Time immediateAnswerRxTimestamp) {


	//1. Timestamp TX lesen, Register 0x17 Timestamp von bit 0-39

	DW1000Time txTimeStamp;
	DW1000.getTransmitTimestamp(txTimeStamp);

	//2. Timestamp RX lesen, Register 0x15 Timestamp von bit 0-39
	DW1000Time rxTimeStamp = immediateAnswerRxTimestamp;


	DW1000Time processingTime;
	processingTime = rxTimeStamp - txTimeStamp;

	//4. an requester schicken


	st_message_t processingTimeMessage;
	processingTimeMessage.targetID = id_requester;
	processingTimeMessage.time = processingTime;
	processingTimeMessage.messageType = PROCESSING_TIME;

	dwm1000_SendData(&processingTimeMessage);
}

DW1000Time dwm1000_getTxTimestamp() {
	DW1000Time ret;
	DW1000.getTransmitTimestamp(ret);

	return ret;
}

DW1000Time dwm1000_getRxTimestamp() {
	DW1000Time ret;
	DW1000.getTransmitTimestamp(ret);

	return ret;
}

e_interrupt_type_t dwm1000_EvalInterrupt()
{
	//zurücksetzen
	lastInterrupt = FAILED_EVAL;

	//funktionen callen, welche je nach statusregisterzustand entsprechende handler callt
	DW1000.handleInterrupt();

	//von handlern gesetzte werte zurückgeben
	return lastInterrupt;
}


void __attribute__((used)) EXTI11_Callback(void)
{
	DMW1000_IRQ_Flag = 1;	//aktiviert synchrone "ISR" in ai_task.c
}




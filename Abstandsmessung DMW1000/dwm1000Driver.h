/*
ai_dwm1000_driver.h

Dieser Header beinhaltet die Funktionen, welche benoetigt werden, um den DWM1000 zu steuern (Daten Senden, Ranging, ...).
Zusaetzlich werden die wichtigen Registerkonfigurationen defined und beschrieben.

Sollten keine Funktionsaenderungen des DWM1000 gewuenscht sein, sollte dieser Driver am Besten NICHT geaendert werden!
*/

#ifndef _DWM1000DRIVER_h
#define _DWM1000DRIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "ai_datatypes.h"
#include "ai_config.h"
#include <SPI.h>
#include <DW1000Time.h>
#include <DW1000Ranging.h>
#include <DW1000Mac.h>
#include <DW1000Device.h>
#include <DW1000Constants.h>
#include <DW1000CompileOptions.h>
#include <DW1000.h>

#pragma region Masks
#define BaudRate SPI_BAUDRATE_3MHZ		//hier auch 11.5, 5.25, 2.625, 1.3125 auswaehlbar -- 21MHZ --> (uint16_t)0x0008
#define SPI_BAUDRATE_21MHZ  SPI_BaudRatePrescaler_4     // 21MHz
#define SPI_BAUDRATE_12MHZ  SPI_BaudRatePrescaler_8     // 11.5MHz
#define SPI_BAUDRATE_6MHZ   SPI_BaudRatePrescaler_16    // 5.25MHz
#define SPI_BAUDRATE_3MHZ   SPI_BaudRatePrescaler_32    // 2.625MHz
#define SPI_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_64    // 1.3125MHz


//--------------------------------Instructions:
/*Jede Instruction ist ein Byte gross und wie folgt beschrieben:
Bit 7: Read - 0, Write - 1
Bit 6: 0
Bit 5 - Bit 0: Reg ID*/


#define READ_TX_TIMESTAMP 0b00010111		//Register 0x17
#define READ_RX_TIMESTAMP 0b00010101		//Register 0x15

#define READ_TFC 0b00001000
#define WRITE_TFC 0b10001000

#define WRITE_TXBUFFER 0b10001001

#define READ_SESR 0b00001111				//System Event Status Register

#define READ_SYS_CTRL 0b00001101
#define WRITE_SYS_CTRL 0b10001101

#define READ_SYS_EVENT_MASK 0b00001110 
#define WRITE_SYS_EVENT_MASK 0b10001110 

#define READ_SYS_STATUS 0b00001111
#define WRITE_SYS_STATUS 0b10001111

#define READ_RXBUFFER 0b00010001			//Reg ID:0x11
#define READ_RXTIMESTAMP 0b00010101			//REG ID: 0x15

//--------------------------------Register:

//Register "Transmit Frame Control" (TFC)
/*Instruction Manual S.25 und ab S.74
Register-ID: 0x08
Groesse: 5 Byte
Beschreibung: Register zur Configuration der DWM100 Transmission und Datenpakete
Orientierung: Bit 39, Bit 38, ..., Bit 0
Setzten der einzelnen Bits
(Hier werden zu Veranschaulichung alle Bits nacheinander gesetzt und anschliessen addiert)
Binaer -> Hex
Byte 4	 Byte 3	  Byte 2   Byte 1	Byte 0
00000000 00000000 00000000 00000000 00001100 -> 0x000000000C - TFLEN, Transmission Frame Length (0-6)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TFLE, Transmit Frame Length Extension (7-9)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - R, Reserved (10-12)
00000000 00000000 00000000 00100000 00000000 -> 0x0000002000 - TXBR, Transmit Bit Rate (13+14)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TR, Transmit Ranging enable  (15)
00000000 00000000 00000001 00000000 00000000 -> 0x0000010000 - TXPRF, Transmit Pulse Repetition Frequency (16+17)
00000000 00000000 00000100 00000000 00000000 -> 0x0000040000 - TXPSR, Transmit Pramble Symbol Repititions  (18+19)
00000000 00000000 00010000 00000000 00000000 -> 0x0000100000 - PE, Preamble Extension (20+21)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TXBOFFS, Transmit Buffer index offset (22-31)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - IFSDELAY, Extension f�r TXBOFFS  (32-39)
Addiert und in Hex:
00000000 00000000 00010101 00100000 00001100 -> 0x000015200C - TFC

Muss in der INIT initialisiert werden!
*/
#define WRITE_INIT_TX_FCTRL 0x000015200C;

//Register "System Control Register" (SYS_CTRL)
/*Instruction Manual S.25 und ab S.79
Register-ID: 0x0D
Groesse: 4 Byte
Beschreibung: Register, welches Bits zum Starten und Stoppen der Transmision enthaelt
Orientierung: Bit 31, Bit 30, ..., Bit 0
Binaer -> Hex
z.B. 00000000 00000000 00000000 00000011  -> 0x00000003
Wichtige Bits:
Bit 0: Suppress Checksumme
Bit 1: Transmit Start
Bit 3: Cancel Surppression (=0)
Bit 6: Transmit Off
Bit 8: Enable Receiver

Keine Init nötig, wird mit 0x00000003 beschrieben, um Sendung zu starten!
*/
#define READ_SYS_CTRL_MEMSPACE 0x00000000; 

#define WRITE_TRANSMIT_BITs 0x00000003;	//hiermit OR, dann Transmit Bit auf jeden Fall gesetzt

#define WRITE_RECEIVE_ENABLE 0x00000100 //Nach lesen einer Nachricht aus den Receive Buffer muss dieses Bit gesetzt werden, um neue Nachrichten empfangen zu koennen

//Register "System Event Status Register"
/*Instruction Manual S.25 und ab S.79
Register-ID: 0x0F
Groesse: 5 Byte
Beschreibung:
Orientierung: Bit 39, Bit 38, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erhaelt das 5-Byte grosse Register
Binaer -> Hex
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000

Es koennten folgende interupts gesetzt werden, die für uns relevant sind:
TXFS (Bit 7)-> Transmit Frame Sent, Transmission Done Interrupt Event
RXDFR (Bit 13) -> Receiver Data Frame Ready, wenn die Nachricht fertig gesendet ist
*/
#define READ_SYS_STATUS_MEMSPACE 0x0000000000 //System Event Status Register, Register-ID: 0x0F, Diesen Wert einem Speicherbereich zuschreiben, dann den Inhalt des Registers darauf lesen

#define MESSAGE_RECEIVED_STATUS 0x0000002000;	//Wenn bei verunden mit diesem Wert und dem System Status Register (0x0F) > 0 rauskommt -> Nachricht erhalten

//--------------------------------Masks:
/*Maske für jeden relevanten Interrupt-Typ
Vergleich mit System Event Status Register (Register 0x0F), falls Verundung größer 0 -> abgefragtes Event hat stattgefunden
Gleiche Bits wie bei System Event Status Register setzen( Bit7 und Bit 13)
*/
#define WRITE_INIT_SYS_EVENT_MASK 0x00002080		//Register Sys Event Mask 0x0E

#define SET_TRANSMIT_START WRITE_TRANSMIT_BITs
#define MASK_TRANSMIT_FRAME_SENT 0x80			//Bit 7
#define MASK_RECEIVE_DATA_FRAME_READY 0x2000			//Bit 13

//--------------------------------Defines fuer Hardware Init:
//defines EXTI (Externer Interrupt)
#define EXTI_LineN 	EXTI_Line11		//bestimmt exti input port (von Bitcraze RX genannt)
#define EXTI_Mode_Interrupt 0x00	//interrupt mode - aus stm32f4xx_exti.h
#define EXTI_Trigger_Rising 0x08	//aus stm32f4xx_exti.h
#define ENABLE 0x01

//defines fuer SPI (Serial Peripheral Interface, Bussystem fuer Kommunikation mit DWM1000)
#define CS_PIN DECK_GPIO_IO1		//CS/SS Pin ist "IO_1"
#define GPIO_Mode_OUT 0x01
#define GPIO_OType_OD 0x01
#define GPIO_PIN_IRQ GPIO_Pin_11
#define GPIO_PIN_RESET GPIO_Pin_10
#define GPIO_PORT GPIOC

//defines fuer NVIC (Nested Vectored Interrupt Controller, ARM Interrupt Handling)
#define EXTI_IRQChannel EXTI15_10_IRQn

#define LOW 0x0
#define HIGH 0x1
#define RX_TIMEOUT 1000


#pragma endregion


//--------------------------------Functions:
bool setup_dwm1000_communication();
/*
initialisiert das SPI interface (nach Anleitung durch "stm32f4xx_spi.c" - Z.17 - Z.134)
*/

bool dwm1000_SendData(st_message_t *message);
/*
sendet Daten ueber den UWB-Bus

data - bytehaufen, der gesendet wird
lengthOfData - Laenge der Daten in byte
(return true, wenn geglueckt)
*/


e_message_type_t dwm1000_ReceiveData(st_message_t *data);
/*
liest Daten im dwm1000 Receivebuffer

data - Stelle, an die die Daten geschrieben werden koennen
lengthOfData - Laenge der Daten die vom Receivebuffer gelesen werden sollen
(return true, wenn geglueckt)
*/

void dwm1000_requestDistance(char nameOfOtherDWM);
/*
startet Rangingvorgang aus Sicht DWM1000

nameOfOtherDWM - PAN (Personal Area Network) Identifier (8Byte) des anderen DWMs (Register 0x03)
*/

e_interrupt_type_t dwm1000_EvalInterrupt();
/*Gibt zurueck aus welchem Grund der Interrupt (IRQ) ausgeloest wurde*/

void dwm1000_sendProcessingTime(char id_requester);
/*Sendet die Zeit die zwischen Eingang der Request-Distance und Immediate-Answer vergangen ist*/

//void dwm1000_init();
/*
beschreibt alle Register mit den in "newConfig" f�r diese enthaltenen Werten

newConfig - Konfigurationsstrukt, das die Werte der Register enthaelt
return die Werte der aktuellen Konfiguration des DWM1000s um Pr�fen zu koennen ob init geglueckt
*/

void dwm1000_immediateDistanceAnswer(char id_requester);

dwTime_t dwm1000_getRxTimestamp();

dwTime_t dwm1000_getTxTimestamp();

#endif





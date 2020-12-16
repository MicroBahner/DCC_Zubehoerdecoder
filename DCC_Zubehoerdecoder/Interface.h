/* Interface des Zubehördecoders
** Alle Interface-abhängigen ( Loconet oder DCC ) Programmkomponenten werden hier
** zusammengefasst, und neutrale Aufrufe für die Funktionalitäten im Sketch zur Verfügung gestellt.
*/

#ifndef INTERFACE_H
#define INTERFACE_H

#include <inttypes.h>
#include <Arduino.h>
#include "src/DebugDefs.h"

//#define LOCONET // Wird dies auskommentiert, wird ein DCC-Interface eingebunden

//######################################################################################################
#define NC 0xff    // nicht verwendeten Funktionsausgängen kann der Port NC zugeweisen werden.
// Da die Prüfung auf ungültige Pin-Nummern in den Arduino-internen Implementierungen je nach Prozessor
// unterschiedlich ist, wird im Sketch selbst auf NC geprüft, und gegebenenfalls die Arduino Funktion nicht aufgerufen.

//---------- defines für LocoNet-Interface -----------------------------------------------------------
#ifdef LOCONET
    const uint8_t txPin        =   2;
    // Das Empfangssignal MUSS auf dem pin 4 ( bei Micro/Leonardo ) oder pin 48 ( bei Mega ) liegen.
    
    void ifc_init( uint8_t cvPomLow ) ;
    
//----------- defines für DCC-Interface -----------------------------------------------------------
#else
    #if defined ( ARDUINO_MAPLE_MINI )
        // Definitonen für den MapleMini ( mit STM32 Prozessor )
        const byte dccPin       =   3;
        const byte ackPin       =   18;
    #elif defined (ARDUINO_GENERIC_STM32F103C)
        // Definitionen für generische STM32-Boards 
        const byte dccPin       =   PB0;
        const byte ackPin       =   PB4;
    #else
        // Definitionen für die 'standard' Arduinos ( UNO, Nano, Mega, Micro, Leonardo )
        const uint8_t dccPin       =   2;
        const uint8_t ackPin       =   4;
    #endif
#endif

//----------- allgemeine defines für beide Interfaces ----------------------------------------------
// Modes für progMode
extern byte progMode;
#define NORMALMODE  0
#define ADDRMODE    1   // Warte auf 1. Telegramm zur Bestimmung der ersten Weichenadresse ( Prog-Led blinkt )
#define PROGMODE    2   // Adresse empfangen, POM-Programmierung m�glich ( Prog-Led aktiv )
#define POMMODE     3   // Allways-Pom Mode ( keine Prog-Led )
#define INIMODE     4   // Wie Normalmode, aber die StandardCV-Werte und CV47-49 werden bei jedem
                        // Start aus den defaults geladen
#define INIALL      5   // Alle CV's m�ssen initiiert werden

//...............Definition der CV-Adressen ..................................
#define CV_INIVAL     45  // Valid-Flag
#define CV_MODEVAL    47  // Initiierungs-CV,  Bits 0..3 für Betriebsarten
// Da die SV-Adressen ( LocoNet ) und die CV-Adressen (DCC ) von den jeweiligen Libs um zwei versetzt
// adressiert werden, sind die EEPROM-Werte beim Wechsel des Interfaces ungültig. Deshalb muss in diesem
// Fall das EEPROM neu initiiert werden. Wegen des Versatzes um 2 wird das Valid-Flag in 2 
// Speicherzellen geschrieben, so dass das Valid-Flag des jeweils anderen Interfaces sicher zerstört wird.
// VALID-Flag ist ab V7 geändert, damit bei Wechsel von V6 <-> V7 die CV's initiiert werden ( andere Aufteilung )
#ifdef LOCONET
    #define VALIDFLG  0xB0 // Wenn das Interface sich ändert, muss alles neu initiiert werden.
                           // Low Nibble muss 0 sein ( wg. MODEVAL-Bits )
#else
    #define VALIDFLG  0x60
#endif

#define CV_POMLOW     48  // Adresse für die POM-Programmierung
#define CV_POMHIGH    49
#define CV_ADRZAHL    50  // Anzahl der verwalteten Adresse ( nicht änderbar )
#define CV_INITYP     51  // CV51 ff enthalten die Funktionstypen der Adressen ( nicht änderbar )
#define CV_FUNCTION  120  // Start der Blöcke für die Funktionskonfiguration
#define CV_BLKLEN     10  // Länge eines CV-Blocks ( pro Adresse ein Block )
                          // Die Bedeutung ist weitgehend funktionsspezifisch

extern const byte modePin;
#define SET_PROGLED digitalWrite( modePin, HIGH )
#define CLR_PROGLED digitalWrite( modePin, LOW )

extern const uint8_t cvAccDecAddressLow;
extern const uint8_t cvAccDecAddressHigh;
extern const uint8_t cvVersionId;
extern const uint8_t cvManufactId;
extern const uint8_t cv29Config;
extern const uint8_t config29Value;
extern const uint8_t config29AddrMode;
extern const uint8_t manIdValue;

void ifc_notifyDccAccState( uint16_t Addr, uint8_t OutputAddr, uint8_t State );
void ifc_notifyCVAck ( void );
void ifc_notifyCVChange( uint16_t CvAddr, uint8_t Value );
void ifc_notifyCVResetFactoryDefault(void);
void ifc_notifyDccReset( uint8_t hardReset );

void ifc_init( uint8_t version, uint8_t progMode, uint8_t cvPomLow );

uint8_t ifc_getCV( uint16_t address );
void ifc_setCV( uint16_t address, uint8_t value );
uint16_t ifc_getAddr();
void ifc_process();

//======================  allgemeine Hilfsfunktionen ==================================
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode );
#else
void _pinMode( byte port, byte mode );
#endif

void _digitalWrite( byte port, byte state ) ;



#endif

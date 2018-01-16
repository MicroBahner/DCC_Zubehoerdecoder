/* Interface des Zubeh�rdecoders
** Alle Interface-abh�ngigen ( Loconet oder DCC ) Programmkomponenten werden hier
** zusammengefasst, und neutrale Aufrufe f�r die Funktionalit�ten im Sketch zur Verf�gung gestellt.
*/

#ifndef INTERFACE_H
#define INTERFACE_H

#include <inttypes.h>
#include <Arduino.h>
#include "src/DebugDefs.h"

//#define LOCONET // Wird dies auskommentiert, wird ein DCC-Interface eingebunden

//######################################################################################################

//---------- defines für LocoNet-Interface -----------------------------------------------------------
#ifdef LOCONET
    const uint8_t txPin        =   2;
    // Das Empfangssignal MUSS auf dem pin 4 ( bei Micro/Leonardo ) oder pin 48 ( bei Mega ) liegen.
    
    
//----------- defines für DCC-Interface -----------------------------------------------------------
#else
    #if defined ( ARDUINO_MAPLE_MINI )
        const byte dccPin       =   3;
        const byte ackPin       =   18;
    #elif defined (ARDUINO_GENERIC_STM32F103C)
        const byte dccPin       =   PB0;
        const byte ackPin       =   PB4;
    #else
        const uint8_t dccPin       =   2;
        const uint8_t ackPin       =   4;
    #endif
#endif

//----------- allgemeine defines für beide Interfaces ----------------------------------------------
// Modes f�r progMode
extern byte progMode;
#define NORMALMODE  0
#define ADDRMODE    1   // Warte auf 1. Telegramm zur Bestimmung der ersten Weichenadresse ( Prog-Led blinkt )
#define PROGMODE    2   // Adresse empfangen, POM-Programmierung m�glich ( Prog-Led aktiv )
#define POMMODE     3   // Allways-Pom Mode ( keine Prog-Led )
#define INIMODE     4   // Wie Normalmode, aber die StandardCV-Werte und CV47-49 werden bei jedem
                        // Start aus den defaults geladen
#define INIALL      5   // Alle CV's m�ssen initiiert werden

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

void ifc_notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State );
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

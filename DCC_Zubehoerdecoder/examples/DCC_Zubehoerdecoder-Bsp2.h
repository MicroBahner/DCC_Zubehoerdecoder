// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

// Beispiel für Variante mit 4 Servos und statischen/blinkenden LED-Ausgängen
// für Arduino UNO/Nano/Micro

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
// #define FIXMODE NORMALMODE    // Ist dieses define aktiv, wird der Betriebsmode fest gesetzt, betrModeP wird dann
                        // nicht gelesen und ignoriert. Mögliche Werte:
                        // NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A0;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A1;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =   A2;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A3;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const int DccAddr          =  20;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
                                    // mit LocoNet-Schnittstelle ist dies die LocoNetId
//#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist (nur DCC ).
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   17;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT    // Wechselblinker mit beiden Leds an beim Start            
const byte iniTyp[]   = {   FSERVO,   FSERVO,   FSERVO,   FSERVO,  FSTATIC,  FSTATIC,    FCOIL };
const byte out1Pins[] = {        2,        4,        6,        8,        9,       10,       12 }; 
const byte out2Pins[] = {        3,        5,        7,       NC,       NC,       11,       13 };
const byte out3Pins[] = {       NC,       NC,       NC,       NC,       NC,       NC,       NC };

const byte iniFmode[] = { SAUTOOFF, SAUTOOFF, SAUTOOFF,        0,  BLKMODE,        0, CAUTOOFF };
const byte iniPar1[]  = {        0,        0,        0,        0,       50,        0,       50 };
const byte iniPar2[]  = {      180,      180,      180,      180,       50,        0,       50 };
const byte iniPar3[]  = {        8,        8,        8,        0,      100,        0,        0 };
const byte iniPar4[]  = {        0,         0,       0,        0,        0,        0,        0 }; // nur für Lichtsignale!
//------------------------------------------------------------------------------------

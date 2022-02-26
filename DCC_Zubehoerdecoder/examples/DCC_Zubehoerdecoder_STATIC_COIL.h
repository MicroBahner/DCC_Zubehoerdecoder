//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder-LS-Nano.h"  // Pfad zu einer alternativen Konfig-Datei
#define KONFIG_FILE "TestKonf\DCC_ZubehoerdecoderV71.h"  // Pfad zu einer alternativen Konfig-Datei
#if !defined( KONFIG_FILE ) || defined ( EXEC_KONFIG )

// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

#define IFC_SERIAL  Serial  // Ist der define aktiv, können Kommandos auch über die serielle Schnittstelle abgesetzt werden
#define SERIAL_BAUD 115200

// Beispiel für Variante mit Licht-Ausfahrsignal mit Vorsignal, mit Betriebsmode Led an Pin 13 (interne Led)
// für Arduino Nano

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
// #define FIXMODE NORMALMODE    // Ist dieses define aktiv, wird der Betriebsmode fest gesetzt, betrModeP wird dann
                        // nicht gelesen und ignoriert. Mögliche Werte:
                        // NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =   A5;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A4;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV       // CV-Werte ab V7.0 ( 10 CV per Adresse )

const int DccAddr           =  17;    // DCC-Decoderadresse
const byte iniMode          = AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
                                    // mit LocoNet-Schnittstelle ist dies die LocoNetId
//#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define STATICRISE  (250/50 << 4) // Softled riseTime = 250 ( max = 750 )
#define COILMOD     NOPOSCHK|CAUTOOFF
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE    // Wechselblinker mit beiden Leds an beim Start            
const byte iniTyp[]     =   {   FSTATIC,   FSTATIC3,   FCOIL };
const byte out1Pins[]   =   {        A0,   /*rt*/ 9,        3 };  // output-pins der Funktionen
const byte out2Pins[]   =   {        A1,   /*rt*/10,        5 };
const byte out3Pins[]   =   {         7,   /*gn*/11,        6 };

const byte iniCVx[10][sizeof(iniTyp)]  = {
/* iniFmode (CV120,130,..*/ { STATICMOD,    0b00111,  COILMOD },
/* iniPar1 (CV121,131,..*/  {       100,         50,       50 },
/* iniPar2 (CV122,132,..*/  {       100,         25,       50 },
/* iniPar3 (CV123,133,..*/  {       200,    0b00101,        0 },
/* iniPar4 (CV124,134,..*/  {         0,        100,        0 }, 
/* iniPar5 (CV125,135,..*/  {         0,        200,        0 },
/* iniPar6 (CV126,136,..*/  {         0,    0b00101,        0 },
/* iniPar7 (CV127,137,..*/  {         0,        200,        0 },
/* iniPar8 (CV128,138,..*/  {         0,          0,        0 },
/* iniState (CV129,139,..*/ {         0,          0,        0 }}; // Status-Werte
//------------------------------------------------------------------------------------
#endif

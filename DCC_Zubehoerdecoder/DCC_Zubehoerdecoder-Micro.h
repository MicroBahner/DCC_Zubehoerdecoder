// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

// Beispiel für Variante mit Licht-Ausfahrsignal mit Vorsignal, mit Betriebsmode Led an Pin 13 (interne Led)
// für Arduino Micro

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------
// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
// #define FIXMODE NORMALMODE    // Ist dieses define aktiv, wird der Betriebsmode fest gesetzt, betrModeP wird dann
                        // nicht gelesen und ignoriert. Mögliche Werte:
                        // NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A5;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A4;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =   A3;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A2;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const int DccAddr           = 317;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
                                    // mit LocoNet-Schnittstelle ist dies die LocoNetId
//#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden


// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#ifdef V6.2Values
#define STATICRISE  (250/50 << 4) // Softled riseTime = 250
#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE    // Wechselblinker mit beiden Leds an beim Start            
const byte iniTyp[]     =   {    FSTATIC,  FSERVO,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL };
const byte out1Pins[]   =   {       A0,         3,   /*rt*/ 9,   /*rt*/10,  /*ge*/12,       14 };  // output-pins der Funktionen
const byte out2Pins[]   =   {       A1,         5,   /*gn*/11,   /*ws*/ 8,  /*gn*/13,       15 };
const byte out3Pins[]   =   {       NC,        NC,   /*ge*/ 7,         NC,        NC,       NC };
 
const byte iniFmode[]     = {STATICMOD,  SERVOMOD,          0,          0,         0,  COILMOD };
const byte iniPar1[]      = {       50,        30,    0b01001,    0b10001,      0b01,       50 };
const byte iniPar2[]      = {       50,       150,    0b00010,    0b00110,      0b10,       50 };
const byte iniPar3[]      = {       50,         8,          5,          0,        19,        0 };
const byte iniPar4[]      = {        0,         0,    0b00101,          0,         0,        0,}; // nur für Lichtsignale!
//------------------------------------------------------------------------------------
#else
#define STATICRISE  (250/50 << 4) // Softled riseTime = 250
#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT //|SAUTOBACK
#define SERVO0MOD   SERVOMOD    // Modbyte für Folgeservo (FSERVO0)
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE    // Wechselblinker mit beiden Leds an beim Start            
const byte iniTyp[]     =   {    FSERVO,  FSERVO0,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL };
const byte out1Pins[]   =   {       A0,        A1,   /*rt*/ 9,   /*rt*/10,  /*ge*/12,       14 };  // output-pins der Funktionen
const byte out2Pins[]   =   {        3,         5,   /*gn*/11,   /*ws*/ 8,  /*gn*/13,       15 };
const byte out3Pins[]   =   {       NC,        NC,   /*ge*/ 7,         NC,        NC,       NC };
 
const byte iniFmode[]     = {SERVOMOD,  0b11000100,          0,          0,         0,  COILMOD };
const byte iniPar1[]      = {       30,       110,    0b01001,    0b10001,      0b01,       50 };
const byte iniPar2[]      = {       80,       160,    0b00010,    0b00110,      0b10,       50 };
const byte iniPar3[]      = {        8,         8,          5,          0,        19,        0 };
const byte iniPar4[]      = {        0,         0,    0b00101,          0,         0,        0,}; // nur für Lichtsignale!
//------------------------------------------------------------------------------------
#endif

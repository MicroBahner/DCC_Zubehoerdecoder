// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

///////////////////////////////////////////////////////////////////////
//
//        Beispiel für Maple-Mini (STM32F1 Prozessor)
//
///////////////////////////////////////////////////////////////////////
// vom Anwender änderbare Parameter um den Zubehördecoder an die verwendete HW anzupassen

// Beispiel für Variante mit 4 Servos + 3 statischen Ausgängen, mit Betriebsmode Led an Pin 13 (interne Led)

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------
// maple mini: ( ARDUINO_MAPLE_MINI )
#ifdef ARDUINO_MAPLE_MINI

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
const byte betrModeP    =   4;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   5;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital  ---------

// Drehencoder zur Servojustierung ...........
#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =   6;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   7;
#endif

// generic STM32F1
#ifdef ARDUINO_GENERIC_STM32F103C
const byte betrModeP    =   PA7;     // Analogeingang zur Bestimmung des Betriebsmodus. 
const byte resModeP     =   PA6;     // Rücksetzen CV-Werte + Mittelstellung Servos
//#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
const byte encode1P     =   PA5;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   PA4;
#endif

// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const int DccAddr           =  20;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR; // | ROCOADDR;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
//#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   LED_BUILTIN;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

// maple mini: ( ARDUINO_MAPLE_MINI )
#ifdef ARDUINO_MAPLE_MINI
const byte iniTyp[]     =   {    FSTATIC,  FSERVO,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL };
const byte out1Pins[]   =   {        9,        19,   /*rt*/26,   /*rt*/25,  /*ge*/13,       11 };  // output-pins der Funktionen
const byte out2Pins[]   =   {        8,        17,   /*gn*/14,   /*ws*/27,  /*gn*/12,       10 };
const byte out3Pins[]   =   {       NC,        16,   /*ge*/15,         NC,        NC,       NC };
#endif

// Generic STM32F103C 
#ifdef ARDUINO_GENERIC_STM32F103C
const byte iniTyp[]     =   {    FSERVO,  FSERVO0,   FSIGNAL2,   FSIGNAL0,    FVORSIG,   FCOIL ,      FSERVO,  FSERVO0 };
const byte out1Pins[]   =   {      PA2,       PA3,  /*rt*/PA9, /*rt*/PA10, /*ge*/ PA0,    PC14 ,        PB3,       PB3 };  // output-pins der Funktionen
const byte out2Pins[]   =   {      PB5,       PB6, /*gn*/PC13, /*ws*/PA8,  /*gn*/ PA1,    PC15 ,       PB12,      PB8 };
const byte out3Pins[]   =   {       NC,        NC,  /*ge*/PB7,         NC,         NC,      NC ,       PA15,      PB9 };
#endif
// Für andere Boards hier einfügen

#define STATICRISE  ((250/50) << 4) // Softled riseTime = 250
#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE    // Wechselblinker mit beiden Leds an beim Start            
const byte iniFmode[]     = { SERVOMOD,0b11000100,          0,          0,         0, NOPOSCHK, SERVOMOD,  0b11000100 };
const byte iniPar1[]      = {       30,       110,    0b01001,    0b10001,      0b01,       20,         30,       110  };
const byte iniPar2[]      = {       80,       160,    0b00010,    0b00110,      0b10,       50,         80,       160  };
const byte iniPar3[]      = {        8,         8,          0,          0,        19,        0,          8,         8  };
const byte iniPar4[]      = {        0,         0,    0b00101,          0,         0,        0,          0,         0 ,}; // nur für Lichtsignale!

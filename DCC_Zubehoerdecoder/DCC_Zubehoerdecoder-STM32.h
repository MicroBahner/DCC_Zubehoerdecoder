//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder-LS-STM32.h"  // Pfad zu einer alternativen Konfig-Datei
#if !defined( KONFIG_FILE ) || defined ( EXEC_KONFIG )

// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

///////////////////////////////////////////////////////////////////////
//
//        Beispiel für Maple-Mini (STM32F1 Prozessor)
//
///////////////////////////////////////////////////////////////////////
// vom Anwender änderbare Parameter um den Zubehördecoder an die verwendete HW anzupassen

//#define IFC_SERIAL  Serial  // Ist der define aktiv, können Kommandos auch über die serielle Schnittstelle abgesetzt werden
#define SERIAL_BAUD 115200

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
#define EXTENDED_CV       // CV-Werte ab V7.0 ( 10 CV per Adresse )
const int DccAddr           =  1;    // DCC-Decoderadresse
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
const byte iniTyp[]     =   {    FSTATIC,  FSERVO,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL  ,      FSERVO,  FSERVO0};
const byte out1Pins[]   =   {        9,         8,   /*rt*/26,   /*rt*/25,  /*ge*/11,       13 ,         19,       19 };  // output-pins der Funktionen
const byte out2Pins[]   =   {       17,        16,   /*gn*/14,   /*ws*/27,  /*gn*/10,       12 ,         31,       22  };
const byte out3Pins[]   =   {       NC,        NC,   /*ge*/15,         NC,        NC,       NC ,         20,       21  };
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
const byte iniCVx[10][sizeof(iniTyp)]  = {
/* iniFmode (CV120,130,..*/ { SERVOMOD,0b11000100,          0,          0,         0, NOPOSCHK, SERVOMOD,  0b11000100 },
/* iniPar1 (CV121,131,..*/  {       30,       110,    0b01001,    0b10001,      0b01,       20,         30,       110  },
/* iniPar2 (CV122,132,..*/  {       80,       160,    0b00010,    0b00110,      0b10,       50,         80,       160  },
/* iniPar3 (CV123,133,..*/  {        8,         8,          5,          0,        19,        0,          8,         8  },
/* iniPar4 (CV124,134,..*/  {        0,         0,    0b00101,          0,         0,        0,          0,         0  }, 
/* iniPar5 (CV125,135,..*/  {        0,         0,          0,          0,         0,         0,         0,         0 },
/* iniPar6 (CV126,136,..*/  {        0,         0,          0,          0,         0,         0,         0,         0  },
/* iniPar7 (CV127,137,..*/  {        0,         0,          0,          0,         0,         0,         0,         0  },
/* iniPar8 (CV128,138,..*/  {        0,         0,          0,          0,         0,         0,         0,         0  },
/* iniState (CV129,139,..*/ {        0,         0,          0,          0,         0,         0,         0,         0  }   // Status-Werte 
                            };
//------------------------------------------------------------------------------------
#endif

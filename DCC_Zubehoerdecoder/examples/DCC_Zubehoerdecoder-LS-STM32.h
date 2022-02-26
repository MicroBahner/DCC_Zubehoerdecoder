// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

// Beispiel für einen reinen Lichtsognaldecoder
// für STM32F103


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

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
const byte betrModeP    = PA7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     = PA6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
//#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
//#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =  NC;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =  NC;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const int DccAddr          =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
//#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden


// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   33;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define MAX_LEDS 12 // default ist 16. Kann auf die tatsächlich benutzte Zahl reduziert werden, um RAM zu sparen.
                    // Pro Softled werden 19 Byte benötigt

const byte iniTyp[]     =   { FSIGNAL2,   FSIGNAL2, FSIGNAL0,    FSIGNAL2, FSIGNAL0,  FSIGNAL0,   FVORSIG,  FSIGNAL0,   FSIGNAL2 };
const byte out1Pins[]   =   {      PA2,        PB3,      PB6,         PB7,     PA10,      PA14,      PC14,      PA1,        PA15 };  // output-pins der Funktionen
const byte out2Pins[]   =   {      PA3,        PB5,       NC,         PA8,     PC13,        NC,      PC15,       NC,        PB12 };
const byte out3Pins[]   =   {       NC,        NC ,       NC,         PA9,       NC,        NC,       PA0,       NC,          NC };
                                                                                     
const byte iniFmode[]     = {        0,         0, 0b00000000, 0b0000000, 0b00000000,0b00000000,        0,        0,  0b00000000 };
const byte iniPar1[]      = {   0b0001, 0b0000001, 0b00001010, 0b0001100, 0b00000110,0b01000000,   0b0101,   0b1001,      0b0001 };
const byte iniPar2[]      = {   0b0010, 0b0001000, 0b00000000, 0b0010000, 0b00010001,0b00000000,   0b1010,      255,      0b0010 };
const byte iniPar3[]      = {        0,         0,          0,         7,          0,         0,       15,       16,           0 };
const byte iniPar4[]      = {        0, 0b0000000,          0, 0b0000101,          0,         0,        0,        0,           0,}; // nur für Lichtsignale!

/*
const byte iniTyp[]     =   {    FCOIL,   FSIGNAL2, FSIGNAL0,   FVORSIG,   FSERVO,          FSTATIC };
const byte out1Pins[]   =   {       A2,          9,       12,        7,       A1,                5 };  // output-pins der Funktionen
const byte out2Pins[]   =   {       A3,         10,       NC,        8,        3,                6 };
const byte out3Pins[]   =   {       NC,         11,       NC,        NC,       NC,               NC };
 
// Funktionsspezifische Parameter. Diese Parameter beginnen bei CV 50 und pro Funktionsausgang gibt es
// 5 CV-Werte. Die ersten 4 Werte steuern das Verhalten und in der folgenden Tabelle sind Erstinitiierungswerte
// für diese CV's enthalten. Der 5. Wert dient internen Zwecken und wird hier nicht initiiert
// In der Betriebsart 'INIMode' werden Mode und Parx Werte bei jedem Start aus der folgenden Tabelle übernommen
// Die Tabellenwerte müssen an die Typaufteilung ( iniTyp, s.o.) angepasst werden.
const byte iniFmode[]     = { CAUTOOFF, LEDINVERT, 0b00000100,        0,        0,  BLKMODE|BLKSOFT };
const byte iniPar1[]      = {       50, 0b0000010, 0b00000100,   0b0001,        0,               50 };
const byte iniPar2[]      = {       50, 0b0000001, 0b00001001,   0b0010,      180,               50 };
const byte iniPar3[]      = {        0,         4,          8,        8,        8,              100 };
const byte iniPar4[]      = {        0, 0b0000101,          0,        0,        0,                0,}; // nur für Lichtsignale!

//------------------------------------------------------------------------------------
/* die folgenden Werte dienen als Beispiele für sinnvolle Einträge in der obigen Paramtertabelle. 
// Sie werden im Programm nicht direkt verwendet!
// Standardwerte für Servoausgang 
const byte iniServoMode     = SAUTOOFF;     // = (Mode) automatische Pulsabschaltung
const byte iniServoGerade   = 0;     // = Par1;
const byte iniServoAbzw     = 180;   // = Par2;
const byte inispeed = 8;             // = Par3;

// Standardwerte für Puls-Ausgang (Doppelspule)
const byte iniCoilMode     =  CAUTOOFF;    // = (Mode) automatische Pulsbegrenzung eingeschaltet (0=AUS)
const byte iniCoilOn       = 50;    // = (Par1) 500ms Impuls
const byte iniCoilOff      = 20;    // = (Par2) mindestens 2Sec Pause zwischen 2 Pulsen

// Standardwerte für statisch/Blinkende Ausgänge
// sind 2 Ausgänge definiert (out1Pins, out2Pins), arbeiten sie im Gegentakt.
const byte iniStaticMode    = BLINKMODE; // Bit0: Blinken/Statisch
                                         // Bit1: Beim Blinken starten erst beide Leds dann Wechselblinken
                                         // Bit2: mit weichem Auf/Abblenden (Pins müssen PWM-fähig sein)
const byte iniBlinkOn       = 100;   // = (Par1) 1sec ein
const byte iniBlinkOff      = 50 ;   // = (Par2) 0,5sec aus
*/

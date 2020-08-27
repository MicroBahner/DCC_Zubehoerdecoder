// ----------------- DCC-Zubehördecoder ---------------------------------------
// Diese Datei enthält die vom Anwender änderbaren Parameter um den Zubehördecoder an die 
// gewünschte Funktionalität und die verwendete HW anzupassen

// Beispiel für einen reinen Lichtsignaldecoder (Arduino Uno/Nano/Micro)

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
const byte betrModeP    =   A5;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A4;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
//#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
//#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
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
const int DccAddr           =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )


//Konstante für Lichtsignalfunktion
#define SIG_DARK_TIME   300     // Zeit zwischen Dunkelschalten und Aufblenden des neuen Signalbilds
#define SIG_RISETIME    500     // Auf/Abblendezeit

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define MAX_LEDS 12 // default ist 16. Kann auf die tatsächlich benutzte Zahl reduziert werden, um RAM zu sparen.
                    // Pro Softled werden 19 Byte benötigt
const byte iniTyp[]     =   {    FCOIL,   FSIGNAL2, FSIGNAL0,   FVORSIG,  FSIGNAL0,          FSTATIC };
const byte out1Pins[]   =   {       NC,          9,       12,        5,        8,               A0 };  // output-pins der Funktionen
const byte out2Pins[]   =   {       NC,         10,       NC,        6,       NC,                3 };
const byte out3Pins[]   =   {       NC,         11,       NC,        7,       NC,               NC };
 
const byte iniFmode[]     = { CAUTOOFF, LEDINVERT, 0b00000100,        0,        0,  BLKMODE|BLKSOFT };
const byte iniPar1[]      = {       50, 0b0000010, 0b00000100,   0b0101,   0b1001,               50 };
const byte iniPar2[]      = {       50, 0b0000001, 0b00001001,   0b1010,      255,               50 };
const byte iniPar3[]      = {        0,         4,          8,        8,        9,              100 };
const byte iniPar4[]      = {        0, 0b0000101,          0,        0,        0,                0,}; // nur für Lichtsignale!

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

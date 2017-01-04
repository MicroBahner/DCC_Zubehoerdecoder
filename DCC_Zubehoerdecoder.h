/* Universeller DCC - Zubehördecoder
 * 2 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische/blinkende Ausgänge  
 *  - Einstellen der Servoendlagen per Drehencoder. 
 *    Der Drehencoder bezieht sich immer auf die zuletzt gestellte Weichenposition .
 *  - Die Betriebsmodi und Startverhalten wird über die Analogeingänge A6/A7 eingestellt. Dazu 
 *    müssen dort Pullups eingebaut werden. Jenachdem wieweit die Spannung  heruntergezogen wird werden
 *    die Modi eingestellt:
 *     A7:   5V (offen) normaler Betriebsmodus, kein PoM
 *           3,3V (Spannungsteiler 1:2) PoM immer aktiv, Adresse immer aus defaults
 *           1,6V (Spannungsteiler 2:1) 
 *           0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *     A6:   wird A6 auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
 *           Mittellage gebracht. Sobald der Encoder wieder bewegt wird, bewegt sich das Servo wieder
 *           zur vorhergehenden Position.
 *           Ist A6 beim Programmstart auf 0, werden alle CV's auf die Defaults zurückgesetzt
 *                  
 * Eigenschaften:
 * Bis zu 8 (aufeinanderfolgende) Zubehöradressen ansteuerbar
 * 1. Adresse per Programmierung einstellbar
 * 
 *  Das Verhalten der Funktionalität wird über CV-Programmierung festgelegt: 
 *  Bei Servoausgängen die Endlagen und die Geschwindigkeit
 *  bei Doppelspulenantrieben die Einschaltzeit der Spulen.
 *  bei blinkenden Ausgängen das Blinkverhalten
 *  
 *  Aufteilung der CV's:
 *  CV      Bedeutung    
 *  47      Kennung für Erstinitiierung, allgemeine Optionen die für den gesamten Decoder gelten
 *  48/49   Pom-Adresse
 *  50-54   Parameter für 1. Weichenadresse
 *  55-59   Parameter für 2. Weichenadresse
 *  ...
 *  Bedeutung der CV's bei den verschiedenen Funktione (CV-Nummern für 1. Weichenadresse)
 *  Servo:
 *  CV50    Bit0 = 1: AutoOff der Servoimpulse bei Stillstand des Servo
 *  CV51    Position des Servo für Weichenstellung '0' ( in Grad, 0...180 )
 *  CV52    Position des Servo für Weichenstellung '1' ( in Grad, 0...180 )
 *  CV53    Geschwindigkeit des Servo
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  Doppelspulenantrieb: ( derzeit nur mit automatischer Abschaltung )
 *  CV50    Bit0 = 1: Spulenausgang automatisch abschalten
 *               = 0: Spulenausgang über DCC-Befehl abschalten
 *  CV51    Einschaltdauer der Spule 1 ( in 10ms Einheiten )
 *  CV52    minimale Ausschaltdauer der Spule ( in 10ms Einheiten )
 *  CV53    -
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  statischer/Blinkender Ausgang ( derzeit nur statisch möglich )
 *  CV50    Bit0 = 1: Blinken,  0: statisch
 *          Bit1 = 1: Beim Blinken starten erst beide Leds dann Wechselblinken
 *          Bit2: mit weichem Auf/Abblenden (Pins müssen PWM-fähig sein)
 *  CV51    Einschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV52    Ausschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV53    Auf/Abblendezeit
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
*/
#define VARIANTE 3  // aktuell verwendete Variante der Konstanten ( hierüber können in dieser Datei
                    // mehrere Parametersätze gespeichert werden, wobei immer nur einer aktuell gültig ist
                    
#define ENCODER_DOUBLE


// vom Anwender änderbare Parameter um den Zubehördecoder an die verwendete HW anzupassen

/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// VARIANTE 1 ////////////////////////////////////////////////////
//###################################################################################################
// Konstante für Variante mit 3 Servos + 3 statische Ausgänge, mit Betriebsmode Led an Pin 13
#if VARIANTE == 1
//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
// Eingänge digital:
const byte encode1P     =   A4;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A5;

// Eingänge analog:
const byte betrModeP    =   A7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)
const byte out1Pins[]   =   {  A3,  A2,  A1,  12, 11, 10 };  // output-pins der Funktionen
const byte out2Pins[]   =   {   3,   5,   6,  NC, NC,  9 };
const byte iniTyp[]         = { FSERVO  ,   FSERVO,   FSERVO, FSTATIC, FSTATIC, FSTATIC };
// In der Betriebsart 'INIMode' werden Mode und Parx Werte bei jedem Start aus der folgenden Tabelle übernommen
const byte iniFmode[]        = { SAUTOOFF, SAUTOOFF, SAUTOOFF,       0,       0,       0 };
const byte iniPar1[]        = {        0,        0,        0,       0,       0,       0 };
const byte iniPar2[]        = {      180,      180,      180,       0,       0,       0 };
const byte iniPar3[]        = {        8,        8,        8,       0,       0,       0 };
//----------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar )
const byte DccAddr          =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR | ROCOADDR;  // default-Betriebsmodus ( CV47 )
const byte PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )

// Funktionsspezifische Parameter. Diese Parameter beginnen bei CV 50 und pro Funktionsausgang gibt es
// 5 CV-Werte.
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
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// VARIANTE 2 ////////////////////////////////////////////////////
//###################################################################################################
// Konstante für Variante mit 5 Servos + 2 statische Ausgänge, mit Betriebsmode Led an Pin 13
#if VARIANTE == 2
//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
// Eingänge digital:
const byte encode1P     =   A4;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A5;

// Eingänge analog:
const byte betrModeP    =   A7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)
const byte out1Pins[]   =   {  A3,  A2,  A1,  A0,  12, 10, 11 };  // output-pins der Funktionen
const byte out2Pins[]   =   {   3,   5,   6,   7,   8,  9, NC };
const byte iniTyp[]         = {   FSERVO,   FSERVO,   FSERVO,   FSERVO,   FSERVO, FSTATIC, FSTATIC };
// In der Betriebsart 'INIMode' werden Mode und Parx Werte bei jedem Start aus der folgenden Tabelle übernommen
const byte iniFmode[]        = { SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF,       0,       0 };
const byte iniPar1[]        = {        0,        0,        0,        0,        0,       0,       0 };
const byte iniPar2[]        = {      180,      180,      180,      180,      180,       0,       0 };
const byte iniPar3[]        = {        8,        8,        8,        8,        8,       0,       0 };

//----------------------------------------------------------------
// Betriebswerte ( per CV änderbar )
const byte DccAddr          =  9;    // DCC-Weichenadresse ( für erste Funktion )
                                     // Decoder wird im Output-Adress-Mode betrieben
const byte iniMode          = 0x50 | ROCOADDR;  // default-Betriebsmodus ( CV47 )
const byte PomAddr          = 52;    // Adresse für die Pom-Programmierung ( CV48/49 )

// Funktionsspezifische Parameter. Diese Parameter beginnen bei CV 50 und pro Funktionsausgang gibt es
// 5 CV-Werte.
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
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// VARIANTE 3 ////////////////////////////////////////////////////
//###################################################################################################
// Konstante für Variante mit 7 Servos + 1 statischer Ausgang
#if VARIANTE == 3
//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
// Eingänge digital:
const byte encode1P     =   A4;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A5;

// Eingänge analog:
const byte betrModeP    =   A7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)
const byte out1Pins[]   =   {  A3,  A2,  A1,  A0,  12,  11, 10,  9 };  // output-pins der Funktionen
const byte out2Pins[]   =   {   3,   5,   6,   7,  NC,  NC, NC,  8 };
const byte iniTyp[]         = {   FSERVO,   FSERVO,   FSERVO,   FSERVO,   FSERVO,   FSERVO,   FSERVO, FSTATIC };
// In der Betriebsart 'INIMode' werden Mode und Parx Werte bei jedem Start aus der folgenden Tabelle übernommen
const byte iniFmode[]        = { SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF, SAUTOOFF,       0 };
const byte iniPar1[]        = {        0,        0,        0,        0,        0,        0,        0,       0 };
const byte iniPar2[]        = {      180,      180,      180,      180,      180,      180,      180,       0 };
const byte iniPar3[]        = {        8,        8,        8,        8,        8,        8,        8,       0 };

//----------------------------------------------------------------
// Betriebswerte ( per CV änderbar )
const byte DccAddr          =  1;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | ROCOADDR;  // default-Betriebsmodus ( CV47 )
const byte PomAddr          = 53;    // Adresse für die Pom-Programmierung ( CV48/49 )

// Funktionsspezifische Parameter. Diese Parameter beginnen bei CV 50 und pro Funktionsausgang gibt es
// 5 CV-Werte.
// Standardwerte für Servoausgang
const byte iniServoMode     = 0 ; //SAUTOOFF;     // = (Mode) automatische Pulsabschaltung
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
#endif


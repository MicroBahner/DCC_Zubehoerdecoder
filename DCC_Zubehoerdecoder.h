/* Universeller DCC - Zubehördecoder
 * 2 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische/blinkende Ausgänge  
 *  - Einstellen der Servoendlagen per Drehencoder. 
 *    Der Drehencoder bezieht sich immer auf die zuletzt gestellte Weichenposition .
 *  - Die Betriebsmodi und Startverhalten wird über die Analogeingänge A4/A5 (parametrierbar) eingestellt. Dazu 
 *    müssen dort Pullups eingebaut werden. Jenachdem wieweit die Spannung  heruntergezogen wird werden
 *    die Modi eingestellt:
 *     A5:   5V (offen) normaler Betriebsmodus, kein PoM
 *           3,3V (Spannungsteiler 1:2) PoM immer aktiv, Adresse immer aus defaults
 *           1,6V (Spannungsteiler 2:1) 
 *           0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *     A4:   wird A6 auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
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
 *  CV54    aktueller Zusatnd ( nicht manuell verändern! )
 *  
*/
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)


// vom Anwender änderbare Parameter um den Zubehördecoder an die verwendete HW anzupassen

// Beispiel für Variante mit 4 Servos + 3 statischen Ausgängen, mit Betriebsmode Led an Pin 13 (interne Led)

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------
// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
const byte betrModeP    =   A5;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A4;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden):
const byte encode1P     =   A3;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A2;

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)
const byte iniTyp[]     =   { FSERVO, FSERVO, FSERVO, FSERVO, FSTATIC, FSTATIC, FSTATIC };
const byte out1Pins[]   =   {     A0,     A1,     11,     12,       7,       8,      10 };  // output-pins der Funktionen
const byte out2Pins[]   =   {      3,      5,      6,     NC,      NC,      NC,       9 };

//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const byte DccAddr          =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR | ROCOADDR;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )

// Funktionsspezifische Parameter. Diese Parameter beginnen bei CV 50 und pro Funktionsausgang gibt es
// 5 CV-Werte. Die ersten 4 Werte steuern das Verhalten und in der folgenden Tabelle sind Rstinitiierungswerte
// für diese CV's enthalten. Der 5 Wert dient internen Zwecken und wird hier nicht initiiert
// In der Betriebsart 'INIMode' werden Mode und Parx Werte bei jedem Start aus der folgenden Tabelle übernommen
// Die Tabellenwerte müssen an die Typaufteilung ( iniTyp, s.o.) angepasst werden.
const byte iniFmode[]     = { SAUTOOFF, SAUTOOFF, SAUTOOFF,       0,       0,BLINKMODE };
const byte iniPar1[]      = {        0,        0,        0,       0,       0,      100 };
const byte iniPar2[]      = {      180,      180,      180,       0,       0,       50 };
const byte iniPar3[]      = {        8,        8,        8,       0,       0,        0 };

//-------------------------------------------------------------------------------------
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

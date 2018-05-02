/* Universeller DCC - Zubehördecoder
 * 3 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische/blinkende Ausgänge 
 *  - Lichtsignale 
 *  - Einstellen der Servoendlagen per Drehencoder. 
 *    Der Drehencoder bezieht sich immer auf die zuletzt gestellte Weichenposition .
 *  - Die Betriebsmodi und Startverhalten wird über die Analogeingänge A4/A5 (parametrierbar) eingestellt. Dazu 
 *    müssen dort Pullups eingebaut werden. Jenachdem wieweit die Spannung  heruntergezogen wird werden
 *    die Modi eingestellt:
 *     A5:   5V (offen) normaler Betriebsmodus, kein PoM
 *           3,3V (Spannungsteiler 1:2) PoM immer aktiv, Adresse immer aus defaults
 *           1,6V (Spannungsteiler 2:1) IniMode: CV's werden immer auf init-Werte aus .h-Datei gesetzt
 *           0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *     A4:   wird A4 auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
 *           Mittellage gebracht. Sobald der Encoder wieder bewegt wird, bewegt sich das Servo wieder
 *           zur vorhergehenden Position.
 *           Ist A4 beim Programmstart auf 0, werden alle CV's auf die Defaults zurückgesetzt
 *                  
 * Eigenschaften:
 * Mehrere (aufeinanderfolgende) Zubehöradressen ansteuerbar. Die mögliche Zahl von Adressen hängt 
 * von den verfügbaren Digitalausgängen ab.
 * Es können maximal 16 Servos angesteuert werden
 * 
 * 1. Adresse per Programmierung einstellbar
 * 
 *  Das Verhalten der konfigurierten Funktionen wird über CV-Programmierung festgelegt: 
 *  Bei Servoausgängen die Endlagen und die Geschwindigkeit
 *  bei Doppelspulenantrieben die Einschaltzeit der Spulen.
 *  bei blinkenden Ausgängen das Blinkverhalten ( in V3.0 noch nicht realisiert )
 *  
 *  Aufteilung der CV's:
 *  CV      Bedeutung    
 *  47      Kennung für Erstinitiierung, allgemeine Optionen die für den gesamten Decoder gelten
 *  48/49   Pom-Adresse
 *  50-54   Parameter für 1. Weichenadresse
 *  55-59   Parameter für 2. Weichenadresse
 *  ...
 *  Bedeutung der CV's bei den verschiedenen Funktione (CV-Nummern für 1. Weichenadresse)
 *  FSERVO Servo:
 *  CV50    Bit0 = 1: AutoOff der Servoimpulse bei Stillstand des Servo
 *          Bit1 = 1: 'Direct-Mode' auch während der Servobewegung wird auf einen erneuten
 *                    Stellbefehl reagiert, und gegebenenfalls sofort die Drehrichtung geändert
 *          Bit3 = 1: kein Überprüfung auf Servoposition bei Empfang eines DCC-Befehls
 *                    bei AUTOOFF und gleicher Position werden wieder Impulse ausgegeben
 *  CV51    Position des Servo für Weichenstellung '0' ( in Grad, 0...180 )
 *  CV52    Position des Servo für Weichenstellung '1' ( in Grad, 0...180 )
 *  CV53    Geschwindigkeit des Servo
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  FCOIL Doppelspulenantrieb: ( derzeit nur mit automatischer Abschaltung )
 *  CV50    Bit0 = 1: Spulenausgang nur automatisch abschalten
 *               = 0: Spulenausgang auch über DCC-Befehl abschalten
 *          Bit3 = 1: kein Überprüfung auf Weichenposition. Gegebenenfalls wird auch an gleichen
 *                    Anschluss wiederholt ein Puls ausgegeben
 *  CV51    Einschaltdauer der Spule  ( in 10ms Einheiten ) 
 *          0= keine automatische Abschaltung, Bit0 im Modebyte muss 0 sein
 *  CV52    minimale Ausschaltdauer der Spule ( in 10ms Einheiten )
 *  CV53    -
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  FSTATIC statischer/Blinkender Ausgang 
 *  CV50    Bit0 = 1: Blinken,  0: statisch
 *          Bit1 = 1: Beim Blinken starten erst beide Leds dann Wechselblinken
 *          Bit2 = 1: mit weichem Auf/Abblenden 
 *  CV51    Einschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV52    Ausschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV53    1. Einschaltzeit beim Start des Blinkens
 *  CV54    aktueller Zusatnd ( nicht manuell verändern! )
 *  
 *  FSIGNAL2 Lichtsignalfunktion mit 1..3 Weichenadressen 
 *          bei den Folgeadressen ist als Typ FSIGNAL0 einzutragen
 *          Lichtsignale starten beim Einschalten immer im Zustand 0 (Bitmuster CV51)
 *  CV50    Signalmodus: Bit7=1 : invertiert die Softled-Ausgänge (HIGH=OFF) (MobaTools ab V0.9)
 *          Bit 2..0: Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *          und welche Ausgänge weich überblenden (Bit=0)
 *  CV51    Bitmuster der Ausgänge für Befehl 1.Adresse 0 (rot)
 *  CV52    Bitmuster der Ausgänge für Befehl 1.Adresse 1 (grün)
 *  CV53    Index des Vorsignals am gleichen Mast ( 0 …. )
 *  CV54    Bitmuster der Zustände, bei denen das Vorsignal dunkel ist:
 *              Bit 0: Befehl 1.Adresse 0 (rot)
 *              Bit 1: Befehl 1.Adresse 1 (grün)
 *              Bit 2: Befehl 2.Adresse 0 (rot)
 *              u.s.w.
 *  FSIGNAL0 1. Folgeadresse (optional)
 *  CV55    Bit 2.. 0 Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *          und Welche Ausgänge weich überblenden (Bit=0)
 *  CV56    Bitmuster der Ausgänge für Befehl 2.Adresse 0 (rot)
 *  CV57    Bitmuster der Ausgänge für Befehl 2.Adresse 1 (grün)
 *  CV58    reserved
 *  CV59    reserved
 *  FSIGNAL0 2. Folgeadresse (optional)
 *  CV60     Bit 2.. 0 Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *           und Welche Ausgänge weich überblenden (Bit=0)
 *  CV61     Bitmuster der Ausgänge für Befehl 3.Adresse 0 (rot)
 *  CV62     Bitmuster der Ausgänge für Befehl 3.Adresse 1 (grün)
 *  CV63     reserved
 *  CV64     reserved
 *  
 *  FVORSIG Vorsignalfunktion
 *          weitgehend wie FSIGNAL2 ausser:
 *  CV53    low Byte der Adresse des angekündigten Hauptsignals
 *  CV54    high Byte der Adrsse des angekündigten Hauptsignals
  */
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)

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
const byte DccAddr          =  20;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )



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
const byte iniTyp[]     =   {    FSTATIC,  FSERVO,   FSIGNAL2,   FSIGNAL0,    FVORSIG,   FCOIL };
const byte out1Pins[]   =   {      PA2,       PB3,  /*rt*/PA9, /*rt*/PA10, /*ge*/PC14,     PA0 };  // output-pins der Funktionen
const byte out2Pins[]   =   {      PA3,       PB5, /*gn*/PC13,  /*ws*/PA8, /*gn*/PC15,     PA1 };
const byte out3Pins[]   =   {       NC,        NC,  /*ge*/PB7,         NC,         NC,      NC };
#endif
// Für andere Boards hier einfügen

#define STATICMOD    CAUTOOFF|BLKSOFT|BLKSTRT    // Wechselblinker mit beiden Leds an beim Start            
const byte iniFmode[]     = {STATICMOD,  SAUTOOFF,          0,          0,         0, NOPOSCHK };
const byte iniPar1[]      = {       50,        30,    0b01001,    0b10001,      0b01,       20 };
const byte iniPar2[]      = {       50,       150,    0b00010,    0b00110,      0b10,       50 };
const byte iniPar3[]      = {       30,         8,          0,          0,        19,        0 };
const byte iniPar4[]      = {        0,         0,    0b00101,          0,         0,        0,}; // nur für Lichtsignale!



/* Universeller DCC - Zubehördecoder
 **************************************************************************************************
 * Beispieldatei für  L i c h t h a u p t s i g n a l e (ohne Vorsignale am gleichen Mast).  
 * Die Pins sind für den Arduino Nano ausgelegt.
 * Es können 3- oder 5-begriffige Hauptsignale angesteuert werden. Die Anzahl der 
 * Ansteuerbaren Signale hängt von der Zahl der Led's ab:
 * Ohne Zugbeeinflussungsrelais können 6 3-Begriffige (3 Leds) oder 3 4-begriffige Signale ( 5/6 Leds) angesteuert werden
 * Es müssen nur gegebenenfalls die PinZuordnungen angepasst werden. Alle anderen Paramter können
 * unverändert bleiben.
 * Die Leds werden aktiv HIGH angesteuert. Für 16V ausgelegte Lichtsignale müssen über einen invertierenden
 * ULN2803 (o.ä.) angesteuert werden
 * Beim Lichtsignaldecoder ist kein Rückkanal für das Auslesen der CV-Werte vorgesehen
 **************************************************************************************************
 * 
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
 *           1,6V (Spannungsteiler 2:1) IniMode: CV's werden immer auf init-Werte aus .h-Datei gesetzt
 *           0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *     A4:   wird A4 auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
 *           Mittellage gebracht. Sobald der Encoder wieder bewegt wird, bewegt sich das Servo wieder
 *           zur vorhergehenden Position.
 *           Ist A4 beim Programmstart auf 0, werden alle CV's auf die Defaults zurückgesetzt
 *                  
 * Eigenschaften:
 * Bis zu 8 (aufeinanderfolgende) Zubehöradressen ansteuerbar. Je nach verfügbaren Digitalausgängen 
 * sind ggfs auch mehr möglich.
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
 *  CV51    Position des Servo für Weichenstellung '0' ( in Grad, 0...180 )
 *  CV52    Position des Servo für Weichenstellung '1' ( in Grad, 0...180 )
 *  CV53    Geschwindigkeit des Servo
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  FCOIL Doppelspulenantrieb: ( derzeit nur mit automatischer Abschaltung )
 *  CV50    Bit0 = 1: Spulenausgang automatisch abschalten
 *               = 0: Spulenausgang über DCC-Befehl abschalten
 *  CV51    Einschaltdauer der Spule 1 ( in 10ms Einheiten )
 *  CV52    minimale Ausschaltdauer der Spule ( in 10ms Einheiten )
 *  CV53    -
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  FSTATIC statischer/Blinkender Ausgang 
 *  CV50    Bit0 = 1: Blinken,  0: statisch
 *          Bit1 = 1: Beim Blinken starten erst beide Leds dann Wechselblinken
 *          Bit2: mit weichem Auf/Abblenden 
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


// vom Anwender änderbare Parameter um den Zubehördecoder an die verwendete HW anzupassen

// Beispiel für Variante mit Licht-Ausfahrsignal mit Vorsignal, mit Betriebsmode Led an Pin 13 (interne Led)

//----------------------------------------------------------------
// Hardwareabhängige Konstante ( nicht per CV änderbar)
//----------------------------------------------------------------

// Eingänge analog: ( Bei Nano und Mini - Versionen kann hier auch A7 und A6 verwendet werden, um mehr
//                    digital nutzbare Ports freizubekommen.
//                    beim UNO sind A7+A6 nicht vorhanden! )
const byte betrModeP    =   A7;     // Analogeingang zur Bestimmung des Betriebsmodus. Wird nur beim
                                    // Programmstart eingelesen!
const byte resModeP     =   A6;     // Rücksetzen CV-Werte + Mittelstellung Servos

// Eingänge digital (die Ports A0-A5 lassen sich auch digital verwenden): ---------

// Drehencoder zur Servojustierung ...........
//#define ENCODER_AKTIV       // Wird diese Zeile auskommentiert, wird der Encoder nicht verwendet. 
                            // Die Encoder-Ports werden dann ignoriert, und können anderweitig 
                            // verwendet werden.
const byte encode1P     =   NC;     // Eingang Drehencoder zur Justierung.
const byte encode2P     =   NC;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Betriebswerte ( per CV änderbar ) Diese Daten werden nur im Initiierungsmodus in die CV's geschrieben.
// Der Initiierungsmodus lässt sich per Mode-Eingang aktivieren oder er ist automatisch aktiv, wenn keine
// sinnvollen Werte im CV47 stehen.
//-------------------------------------------------------------------------------------------------------
const byte DccAddr          =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden

//Konstante für Lichtsignalfunktion
#define SIG_DARK_TIME   300     // Zeit zwischen Dunkelschalten und Aufblenden des neuen Signalbilds
#define SIG_RISETIME    500     // Auf/Abblendezeit

// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define MAX_LEDS 16 // default ist 16. Kann auf die tatsächlich benutzte Zahl reduziert werden, um RAM zu sparen.
                    // Pro Softled werden 19 Byte benötigt
                   
const byte iniTyp[]     =   { FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0,  FSIGNAL2, FSIGNAL0  };
const byte out1Pins[]   =   {       A0,       NC,       A3,       NC,        3,       NC,        6,       NC,         9,       NC  }; 
const byte out2Pins[]   =   {       A1,       NC,       A4,       NC,        4,       NC,        7,       NC,        10,       NC  };
const byte out3Pins[]   =   {       A2,       NC,       A5,       NC,        5,       NC,        8,       NC,        11,       NC  };
                                                                                                                                  
const byte iniFmode[]     = {        0, 0b000100,        0, 0b000100,        0, 0b000100,        0, 0b000100,         0, 0b000100  };
const byte iniPar1[]      = { 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001,  0b001001, 0b110001  };
const byte iniPar2[]      = { 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110,  0b100010, 0b100110  };
const byte iniPar3[]      = {        0,        0,        0,        0,        0,        0,        0,        0,         0,        0  };
const byte iniPar4[]      = { 0b000101,        0, 0b000101,        0, 0b000101,        0, 0b000101,        0,  0b000101,        0  }; 
//------------------------------------------------------------------------------------


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
 */ 

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
#define ENCODER_DOUBLE  // Eigenschaften des Drehencoders (Impulse per Raststellung)
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
#define EXTENDED_CV       // CV-Werte ab V7.0 ( 10 CV per Adresse )

const int DccAddr          =  17;    // DCC-Decoderadresse
const byte iniMode          = 0x50 | AUTOADDR /*| ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 50;    // Adresse für die Pom-Programmierung ( CV48/49 )
#define NOACK                     // Diese Zeile aktivieren, wenn keine HW zum CV auslesen vorhanden ist
                                    // ( kein Ack-Pin ) Der in Interfac.h definierte Pin wird dann zwar als OUTPUT
                                    // gesetzt, kann aber für beliebige Funktionen in der Tabelle unten genutzt werden


// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   13;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#define MAX_LEDS 16 // default ist 16. Kann auf die tatsächlich benutzte Zahl reduziert werden, um RAM zu sparen.
                    // Pro Softled werden 19 Byte benötigt
                   
const byte iniTyp[]     =   { FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0,  FSIGNAL2, FSIGNAL0 };
const byte out1Pins[]   =   {       A0,       NC,       A3,       NC,        3,       NC,        6,       NC,         9,       NC }; 
const byte out2Pins[]   =   {       A1,       NC,       A4,       NC,        4,       NC,        7,       NC,        10,       NC };
const byte out3Pins[]   =   {       A2,       NC,       A5,       NC,        5,       NC,        8,       NC,        11,       NC };
                                                                                                                                  
const byte iniCVx[10][sizeof(iniTyp)]  = {
/* iniFmode (CV120,130,..*/ {        0, 0b000100,        0, 0b000100,        0, 0b000100,        0, 0b000100,         0, 0b000100 },
/* iniPar1 (CV121,131,..*/  { 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001,  0b001001, 0b110001 },
/* iniPar2 (CV122,132,..*/  { 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110,  0b100010, 0b100110 },
/* iniPar3 (CV123,133,..*/  {        0,        0,        0,        0,        0,        0,        0,        0,         0,        0 },
/* iniPar4 (CV124,134,..*/  { 0b000101,        0, 0b000101,        0, 0b000101,        0, 0b000101,        0,  0b000101,        0 }, 
/* iniPar5 (CV125,135,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/* iniPar6 (CV126,136,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/* iniPar7 (CV127,137,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/* iniPar8 (CV128,138,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/* iniState (CV129,139,..*/ {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 }}; // Status-Werte
//------------------------------------------------------------------------------------


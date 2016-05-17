#include <NmraDcc.h>
#include <MobaTools.h>

/*  Demo: ein universeller Dcc-Zubehördecoder 
 *   Version 0.1 - erstmal nur Servos
 *   Version 0.2 - alternativ auch ohne Programmierschalter nutzbar. PoM ist dann immer aktiv,
 *                  Addressen nur über den Sketch änderbar.
 *                  Adressierung als Board- oder Outputadressierung je nach CV29:6 (1=Outputaddr.)
 *  ----------------------------------------
 * Eigenschaften:
 * Bis zu 8 (aufeinanderfolgende) Zubehöradressen ansteuerbar
 * 1. Adresse per Programmierung einstellbar
 * 
 * 2 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische Ausgänge
 *  - blinkende Ausgänge
 *  
 *  Die Funnktionalität wird über CV-Programmierung festgelegt. Bei Servoausgängen
 *  sind die Endlagen per CV-Wert einstellbar
*/
//------------------------------------------ //
// die NmraDcc - Library gibt es unter https://github.com/mrrwa/NmraDcc/archive/master.zip
// oder, ohne Benutzung des Timer0 (PWM an Pin 5 und 6 funktioniert):
// https://github.com/MicroBahner/NmraDcc/archive/Without-use-of-HW-timers.zip

// ---------  gegebenenfalls anzupassende Konstante ( nicht über CV änderbar) --------------------
// defines zum Einstellen der verwendeten Variante. Ist das jeweilige define aktiv, so wird diese
// Variante erzeugt.
//#define VAR_ALLWAYSPOM       // PoM ist immer aktiv, keine automatische Addressvergabe. PoM-Adresse wird immer
                        // aus dem Programmkonstanten geladen.
//#define VAR_INIADDR     // Die Addressen und der Anschlusstyp werden bei jedem Programmstart initiiert
                        // und in die jeweiligen CV's geschrieben
#define VAR_INISERVO  // Die Servo-Impuse werden beim Programmstart sofort ausgegeben.
                        
#define DEBUG ;                  // Wenn dieser Wert gesetzt ist, werden Debug ausgaben auf dem ser. Monitor ausgegeben

#define DCC_DECODER_VERSION_ID 02
#
// Pin-Festlegungen
const byte progPin = 0 ;    // CV-Programmierung nur möglich, wenn auf 0
const byte isROCO = 0  ;    // wegen unterschiedlicher Weichenadressberechnung bei Roco (sonst = 0)
const byte servoPins[]  =   {   3,   5,   9,   7,  12,  A0,  A2};  // output-pin der Servos
const byte relaisPins[] =   {  11,   6,  10,   8,  13,  A1,  A3};
const byte WeichenZahl = sizeof(servoPins);
const byte dccPin       =   2;
const byte ackPin       =   4;
const byte modePin      =   1;  // leuchtet im Programier(Pom) Modus. Da dies auch der serielle
                                // Ausgang ist, funktinoert die Led nicht im Debug-Modus

//-------------------------------Definition der CV-Adressen ---------------------------------------
// Initiierungswerte ( zunächst nur Servo möglich) :
// Mögliche Funktionstypen je Ausgang. Derzeit sind nur die Servofunktionen implementiert
#define FOFF          0 // Funktionsausgang abgeschaltet
#define FSERVO        1 // Standardservoausgang (Impulse liegen dauerhaft an)
#define FSERVOAUTO    2 // automatische abschaltung der Servoimpulse nach erreichen der Zielposition
#define FDOPPELSPULE  3 // Magnetartikel, Ausgang wird nicht automatisch abgeschaltet
#define FDSAUTO       4 // Magnetartikel, Ausgang ist Momentimpuls
#define FSTATISCH     5 // Der Ausgang wird statisch ein bzw ausgeschaltet
#define FBLINKEN      6 // Der Ausgang Blinkt
#define FMAX          6  

const byte DccAddr          =  1;    // DCC-Decoderadresse
const byte PomAddr          = 50;    // Adresse für die Pom-Programmierung
const byte iniTyp[]         = { FSERVO, FSERVO, FSERVO, FSERVOAUTO, FSERVOAUTO, FSERVOAUTO, FSERVOAUTO, FSERVOAUTO };
const byte iniServoGerade   = 0;     // = Par1;
const byte iniServoAbzw     = 180;   // = Par2;
const byte speed = 8;                // = Par3;


#define CV_START    47  // Startadresse des Decoderspezifischen CV-Blocks
typedef struct {        // Definition der Struktur des decoderspezifischen CV-Blocks
    byte initVal;       // =0x55 wenn die CV-Werte initiiert sind.
    byte PomAddrLow;      // Adresse für die POM-Programmierung
    byte PomAddrHigh;
    struct {
        byte Typ;     // Funktionalität des Ausgangs
        byte Par1;    // Servo: Position AUS ( 0...180 Grad )
                      // DoppelspuleAuto: Einschaltzeit ( 10ms-Schritte )
                      // FBlinken:        Einschaltphase des Blinkens ( 10ms Schritte )
        byte Par2;    // Servo: Position EIN ( 0...180 Grad )
                      // DoppelspuleAuto: Einschaltzeit Ausgang 2 ( 10ms-Schritte )
                      // FBlinken:        Ausschaltphase des Blinkens ( 10ms Schritte )
        byte Par3;    // Servo: Speed
        byte State;   // aktueller Status des Ausgangs (Ein/Aus)
                      // Nach einem Neustart werden die Funktionsausgänge entsprechend eingestellt.
    } Fkt [WeichenZahl];
} CvVar_t;

const CvVar_t *CV = (CvVar_t *) CV_START; //Pointer auf die decoderspezifischen CV-Werte im EEProm

// CV Default-Werte der Standardadressen:
struct CVPair {
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DccAddr%256},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DccAddr/256},
  {CV_VERSION_ID, DCC_DECODER_VERSION_ID},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_29_CONFIG, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE},
};

// ----------------------- Variable ---------------------------------------------------
byte isOutputAddr;              // Flag ob Output-Adressing
word weichenAddr;               // Addresse der 1. Weiche (des 8er Blocks)
byte weicheSoll[WeichenZahl];  // Solllage der Weichen
byte weicheIst[WeichenZahl];   // Istlagen der Weichen
#define GERADE  0x0
#define ABZW    0x1
#define MOVING  0x2               // nur für Ist-Zustand, Bit 1 gesetzt während Umlauf

//int geradePulse[WeichenZahl] ;  // Pulslänge geradeaus
//int abzweigPulse[WeichenZahl];  // Pulslänge abzweigend

byte relaisOut[WeichenZahl];    // Ausgabewerte für die Relais ( 0/1, entspricht Sollwert )

byte progMode;      // Merker ob Decoder im Programmiermodus
#define NORMALMODE  0
#define ADDRMODE    1   // Warte auf 1. Telgramm zur Bestimmung der ersten Weichenadresse
#define PROGMODE    2   // Adresse empfangen, POM-Programmierung möglich

Servo8 weicheS[WeichenZahl];
EggTimer AckImpuls;
EggTimer ledTimer;  // zum Blinken der Programmierled
NmraDcc Dcc;


// für debugging
#ifdef DEBUG
#define DB_PRINT( ... ) { sprintf( dbgbuf,"Dbg: " __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
byte debug;
char dbgbuf[80];
#define SET_PROGLED
#define CLR_PROGLED
#else
#define DB_PRINT ;
#define SET_PROGLED digitalWrite( modePin, HIGH )
#define CLR_PROGLED digitalWrite( modePin, LOW )
#endif

//###################### Ende der Definitionen ##############################
//###########################################################################
void setup() {
    #ifdef VAR_ALLWAYSPOM
    // Pom-Modus ist immer aktiv, Programmierschalter und LED werden nicht genutzt
    progMode=PROGMODE;
    #else
    // PoM-Mode nur bei aktiviertem Programmierschalter
    pinMode( A4,OUTPUT);
    pinMode( modePin, OUTPUT );
    digitalWrite( modePin, LOW );
    // Auf Programmmiermodus prüfen
    pinMode ( progPin, INPUT_PULLUP);
    if ( digitalRead( progPin) == LOW ) 
        progMode = ADDRMODE;
    else
        progMode = NORMALMODE;
        
    delay( 1000 );
    #endif
    
    #ifdef DEBUG
      pinMode( modePin, INPUT );
      Serial.begin(115200); //Debugging
      if ( progMode == NORMALMODE )
        Serial.println( "Normal Start of Program" );
      else
        Serial.println( "PoM Start of Program" );
    #endif
    
    pinMode( ackPin, OUTPUT );
    Dcc.pin( digitalPinToInterrupt(dccPin), dccPin, 1); 
    if ( progMode == NORMALMODE ) {
        Dcc.init( MAN_ID_DIY, 15, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) 0) );
        CLR_PROGLED;
    } else {
        Dcc.init( MAN_ID_DIY, 15, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) &CV->PomAddrLow) );
        # ifndef VAR_ALLWAYSPOM
        SET_PROGLED;
        #endif
    }
    
    // Wenn in initVal kein sinnvoller Wert steht wird alles initiiert.
    // Wird über DCC ein 'factory-Reset' empfangen wird dieser Wert zurückgesetzt, was beim nächsten
    // Start ebenfalls zum initiieren führt.
    if ( Dcc.getCV( (int) &CV->initVal) != 0x55 ) {
        // alles initiieren mit den Defaultwerten
        // Standard-CV's
        for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
                Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
        }
        // Decoderspezifische CV's
        Dcc.setCV( (int) &CV->PomAddrLow, PomAddr%256 );
        Dcc.setCV( (int) &CV->PomAddrHigh, PomAddr/256 );
        for ( byte i = 0; i<WeichenZahl; i++ ) {
            Dcc.setCV( (int) &CV->Fkt[i].Typ, iniTyp[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par1, iniServoGerade );
            Dcc.setCV( (int) &CV->Fkt[i].Par2, iniServoAbzw );
            Dcc.setCV( (int) &CV->Fkt[i].Par3, speed );
            Dcc.setCV( (int) &CV->Fkt[i].State, 0 );
        }
        Dcc.setCV( (int) &CV->initVal, 0x55 );
    } //--- Ende Grundinitiierung ---------------------------------

    // Adressmodus aus CV29 auslesen
    isOutputAddr = Dcc.getCV( CV_29_CONFIG ) & CV29_OUTPUT_ADDRESS_MODE;
    // Adresse der 1. Weiche aus Decoderaddresse berechnen
    if ( isOutputAddr ) 
        weichenAddr = Dcc.getAddr( );
    else
        weichenAddr = (Dcc.getAddr( )-1)*4 +1 +isROCO;
        
    DBprintCV(); // im Debug-Mode alle CV-Werte ausgeben
    
    for ( byte i=0; i<WeichenZahl; i++ ) {
        // Funktionen initiieren
        byte autoOff = 0;
        weicheIst[i] = Dcc.getCV( (int) &CV->Fkt[i].State );
        weicheSoll[i] = weicheIst[i];
        switch ( Dcc.getCV(  (int) &CV->Fkt[i].Typ ) ) {
          case FSERVOAUTO:
            autoOff = 1;
          case FSERVO:
            weicheS[i].attach( servoPins[i], autoOff );
            weicheS[i].setSpeed( Dcc.getCV(  (int) &CV->Fkt[i].Par3 ) );
            pinMode( relaisPins[i], OUTPUT );
            #ifdef VAR_INISERVO
            // Servowerte und Relaisausgang initiieren und ausgeben
                if ( weicheSoll[i] == GERADE ) {
                    weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par1 ) );
                } else {
                    weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par2 ) );
                }
                weicheIst[i] = weicheSoll[i];
                relaisOut[i] = weicheIst[i];
                digitalWrite( relaisPins[i], relaisOut[i] );
            #endif
            break;
        }
    }
}
////////////////////////////////////////////////////////////////
void loop() {
    if (digitalRead( A4)) digitalWrite(A4,LOW); else digitalWrite(A4,HIGH); // Test Zykluszeit
    
    Dcc.process(); // Hier werden die empfangenen Telegramme analysiert und der Sollwert gesetzt
    #ifdef DEBUG
    // Merker CV für CV-Ausgabe rücksetzen
    Dcc.setCV( (int) &CV->Fkt[WeichenZahl].Typ , 0xff );
    #endif
    // Servos und Relais ansteuern
    for ( byte i=0; i<WeichenZahl; i++ ) {
        switch ( Dcc.getCV( (int) &CV->Fkt[i].Typ ) ) {
          case FSERVO:
          case FSERVOAUTO: // Servoausgänge ansteuern
            if ( weicheIst[i] & MOVING ) {
                // Weiche wird gerade ungestellt, Schaltpunkt Relais und Bewegungsende überwachen
                if ( weicheS[i].moving() < 50 ) relaisOut[i] = weicheIst[i]& 0x1;
                if ( weicheS[i].moving() == 0 ) {
                    // Bewegung abgeschlossen, 'MOVING'-Bit löschen und Lage in CV speichern
                    weicheIst[i] &= 0x1; 
                    Dcc.setCV( (int) &CV->Fkt[i].State, weicheIst[i] );
                }
            } else if ( weicheSoll[i] != weicheIst[i] ) {
                // Weiche muss umgestellt werden
                DB_PRINT( "WeicheIx=%d stellen, Ist=%d,Soll=%d", i, weicheIst[i], weicheSoll[i] );
                weicheIst[i] = weicheSoll[i] | MOVING; // Istwert auf Sollwert und MOVING-Bit setzen.
                if ( weicheSoll[i] == GERADE ) {
                    weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par1 ) );
                } else {
                    weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par2 ) );
                }
            }
        }
    }

    // Relaisausgänge setzen
    for ( byte i=0; i<WeichenZahl; i++ ) {
        digitalWrite( relaisPins[i], relaisOut[i] );
    }

    if ( !AckImpuls.running() ) digitalWrite( ackPin, LOW );

    #ifndef VAR_ALLWAYSPOM
    // Programmierled blinkt im Programmiermode nach dem Empfang einer Adresse
    if ( ! ledTimer.running() && progMode == PROGMODE) {
        ledTimer.setTime( 500 );
        if ( digitalRead( modePin ) ) 
            CLR_PROGLED;
        else
            SET_PROGLED;
    }
    #endif
}
//////////////////////////////////////////////////////////////
// Unterprogramme, die von der DCC Library aufgerufen werden:
//------------------------------------------------
// Die folgende Funktion wird von Dcc.process() aufgerufen, wenn ein Weichentelegramm empfangen wurde
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State ){
    // Weichenadresse berechnen
    word wAddr = Addr+isROCO; // Roco zählt ab 0, alle anderen lassen die ersten 4 Weichenadressen frei
    // Im Programmiermodus bestimmt das erste empfangen Programm die erste Weichenadresse
    if ( progMode == ADDRMODE ) {
        // Adresse berechnen und speichern
        if (isOutputAddr ) {
            weichenAddr = wAddr;
            Dcc.setCV( CV_ACCESSORY_DECODER_ADDRESS_LSB, wAddr%256 );
            Dcc.setCV( CV_ACCESSORY_DECODER_ADDRESS_MSB, wAddr/256 );
        } else {
            Dcc.setCV( CV_ACCESSORY_DECODER_ADDRESS_LSB, BoardAddr%64 );
            Dcc.setCV( CV_ACCESSORY_DECODER_ADDRESS_MSB, BoardAddr/64 );
            weichenAddr = (Dcc.getAddr( )-1)*4 +1 +isROCO;
        }
        progMode = PROGMODE;
        DB_PRINT( "Neu: Boardaddr: %d, 1.Weichenaddr: %d", BoardAddr, weichenAddr );
    }
    // Testen ob eigene Weichenadresse
    DB_PRINT( "DecAddr=%d, Weichenadresse: %d , Ausgang: %d, State: %d", BoardAddr, wAddr, OutputAddr, State );
    for ( byte i = 0; i < WeichenZahl; i++ ) {
        if (  wAddr == weichenAddr+i ) {
            // ist eigene Adresse, Sollwert setzen
            weicheSoll[i] =  OutputAddr & 0x1;
            DB_PRINT( "Weiche %d, Index %d, Status %d", wAddr, i, weicheSoll[i] );
            break; // Schleifendurchlauf abbrechen, es kann nur eine Weiche sein
        }
    }
}
//---------------------------------------------------
// wird aufgerufen, wenn die Zentrale ein CV ausliest. Es wird ein 60mA Stromimpuls erzeugt
void notifyCVAck ( void ) {
    // Ack-Impuls starten
    //DB_PRINT( "Ack-Pulse" );
    AckImpuls.setTime( 6 );
    digitalWrite( ackPin, HIGH );
}
//-----------------------------------------------------
// Wird aufgerufen, nachdem ein CV-Wert verändert wurde
void notifyCVChange( uint16_t CvAddr, uint8_t Value ) {
    // Es wurde ein CV verändert. Ist dies eine aktive Servoposition, dann die Servoposition
    // entsprechend anpassen
    DB_PRINT( "neu: CV%d=%d", CvAddr, Value );
    for ( byte i=0; i<WeichenZahl; i++ ) {
        // prüfen ob Ausgang einen Servo ansteuert:
        switch ( Dcc.getCV( (uint16_t) &CV->Fkt[i].Typ ) ) {
          case FSERVO:
          case FSERVOAUTO:
            // gehört der veränderte CV zu diesem Servo?
            if (  (CvAddr == (uint16_t) &CV->Fkt[i].Par1 && weicheSoll[i] == GERADE) ||
                  (CvAddr == (uint16_t) &CV->Fkt[i].Par2 && weicheSoll[i] == ABZW ) ){
                // Es handelt sich um die aktuelle Position des Servos,
                // Servo neu positionieren
                DB_PRINT( "Ausg.%d , Pos. %d neu einstellen", i, weicheSoll[i] );
                 weicheS[i].write( Value );
            } else if ( CvAddr == (uint16_t) &CV->Fkt[i].Par3 ) {
                // die Geschwindigkeit des Servo wurde verändert
                weicheS[i].setSpeed( Value );
            }
        }
    }
    #ifdef DEBUG
        // prüfen ob die CV-Adresse HINTER den Weichenadressen verändert wurde. Wenn ja,
        // alle CV-Werte ausgeben und Wert wieder auf 0xff setzen
        if ( CvAddr ==  (int) &CV->Fkt[WeichenZahl].Typ && Value !=0xff ) {
            DBprintCV();
        }
        
    #endif
}    
//-----------------------------------------------------
void notifyCVResetFactoryDefault(void) {
    // Auf Standardwerte zurücksetzen. Hier wird nur ein Merker gesetzt, das eigentliche Rücksetzen findet
    // beim nächsten Neustart statt.
    Dcc.setCV( (int) &CV->initVal, 255 );
    DBprintCV();
}
//------------------------------------------------------
#ifdef DEBUG
void notifyDccReset( uint8_t hardReset ) {
    if ( hardReset > 0 ) DB_PRINT("Reset empfangen, Value: %d", hardReset);
    // wird bei CV-Auslesen gesendet
}
#endif
//--------------------------------------------------------
void softReset(void){
;asm volatile ("  jmp 0");
}
#ifdef DEBUG
void DBprintCV(void) {
    // für Debug-Zwecke den gesamten genutzten CV-Speicher ausgeben
    // Standard-Adressen
    DB_PRINT ("--------- Debug-Ausgabe CV-Werte ---------" );
    DB_PRINT ("Version: %d, ManufactId: %d", Dcc.getCV( CV_VERSION_ID ), Dcc.getCV( CV_MANUFACTURER_ID ) );
    DB_PRINT ("Konfig   (CV29)  : 0x%X", Dcc.getCV( CV_29_CONFIG ) );
    DB_PRINT ("Adresse: (CV1/9) : %d", Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_LSB )+Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_MSB )*256);
    DB_PRINT ("1.Weichenaddresse: %d", weichenAddr );
    
    // Decoder-Konfiguration global
    DB_PRINT ( "Initierungswert: 0x%x ", Dcc.getCV( (int) &CV->initVal ) );
    DB_PRINT ( "PoM-Adresse    : %d"   , Dcc.getCV( (int) &CV->PomAddrLow) + 256* Dcc.getCV( (int) &CV->PomAddrHigh ) );
    
    // Output-Konfiguration
    DB_PRINT( "Wadr | CV's  | Typ | Par1 | Par2 | Par3 | Status |");
    for( byte i=0; i<WeichenZahl; i++ ) {
        DB_PRINT( "%4d | %2d-%2d |%4d | %4d | %4d | %4d | %3d ", weichenAddr+i, &CV->Fkt[i].Typ,  &CV->Fkt[i].State,
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].Typ ),
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].Par1 ),
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].Par2 ),
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].Par3 ),
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].State ) );
    }
    
}
#else
void DBprintCV(void) {
    
}
#endif


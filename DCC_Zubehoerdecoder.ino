#include <NmraDcc.h>
#include <MobaTools.h>

/*  Demo: ein universeller Dcc-Zubehördecoder 
 *   Version 0.1 - erstmal nur Servos
 *   Version 0.2 - alternativ auch ohne Programmierschalter nutzbar. PoM ist dann immer aktiv,
 *                  Addressen nur über den Sketch änderbar.
 *                  Adressierung als Board- oder Outputadressierung je nach CV29:6 (1=Outputaddr.)
 *                  Ansteuerung von Doppelspulenantrieben 
 *   Version 0.2A   Einstellen der Servoendlagen per Drehencoder. Wegen der 2 Encodereingänge
 *                  können maximal 6 Weichen angesteuert werden.
 *                  Der Drehencoder bezieht sich immer auf die zuletzt gestellte Weiche.
 *  ----------------------------------------
 * Eigenschaften:
 * Bis zu 8 (aufeinanderfolgende) Zubehöradressen ansteuerbar
 * 1. Adresse per Programmierung einstellbar
 * 
 * 2 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische Ausgänge   (noch nicht realisert)
 *  - blinkende Ausgänge   (noch nicht realisert)
 *  
 *  Die Funnktionalität wird über CV-Programmierung festgelegt. Bei Servoausgängen
 *  sind die Endlagen per CV-Wert einstellbar
*/
#define DCC_DECODER_VERSION_ID 02

//------------------------------------------ //
// die NmraDcc - Library gibt es unter https://github.com/mrrwa/NmraDcc/archive/master.zip
// oder, ohne Benutzung des Timer0 (PWM an Pin 5 und 6 funktioniert):
// https://github.com/MicroBahner/NmraDcc/archive/Without-use-of-HW-timers.zip

// vvvvvvvvvvvvvv  gegebenenfalls anzupassende Konstante vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

// -------------feste Programmeinstellungen ( nicht über CV änderbar) -----------------------
// Eine Änderung dieser Parameter hat i.d.Regel auch Änderungen an der HW zur Folge

#define VAR_ALLWAYSPOM  0   // 0: PoM muss über den Programmierschalter aktiviert werden
                            // 1: PoM ist immer aktiv, keine automatische Addressvergabe. 
                            //    PoM-Adresse wird immer aus den Factory-Defaults geladen.
#define VAR_INIADDR     0   // 0: Decoder und PoM-Adresse werden nur dann aus den Factory-defaults geladen,
//  noch nicht realisiert   //    wenn CV47 einen ungültigen Wert enthält
                            // 1; Decoder und PoM-Adress werden bei jedem Programmstart aus den Factory
                            //    defaults geladen

#define DEBUG ;             // Wenn dieser Wert gesetzt ist, werden Debug ausgaben auf dem ser. Monitor ausgegeben

#
// Pin-Festlegungen
#ifndef VAR_ALLWASPOM 
const byte progPin      =   0 ;    // PoM-Programmierung nur möglich, wenn auf 0 (ungenutzt, wenn ALLWAYSPOM)
const byte modePin      =   1;  // leuchtet im Programier(Pom) Modus. Da dies auch der serielle
                                // Ausgang ist, funktinoert die Led nicht im Debug-Modus
#endif
const byte dccPin       =   2;
const byte ackPin       =   4;
const byte encode1P     =   A2; // Eingang Drehencoder zur Justierung.
const byte encode2P     =   A3;
const byte out1Pins[]   =   {   3,   5,   9,   7,  12,  A0};  // output-pins der Funktionen
const byte out2Pins[]   =   {  11,   6,  10,   8,  13,  A1};
// Mögliche Funktionstypen je Ausgang. Derzeit sind nur die Servofunktionen implementiert
#define FOFF        0 // Funktionsausgang abgeschaltet
#define FSERVO      1 // Standardservoausgang (Impulse liegen dauerhaft an)
#define FCOIL       2 // Magnetartikel
#define FSTATIC     3 // Der Ausgang wird statisch ein bzw ausgeschaltet
#define FBLINK      4 // Der Ausgang Blinkt
#define FMAX        4  
const byte iniTyp[]         = { FSERVO, FSERVO, FSERVO, FCOIL, FCOIL, FCOIL, FCOIL, FCOIL };
const byte WeichenZahl = sizeof(out1Pins);

//---------------- Initiierungswerte ( Factory-Defaults, per CV änderbar ) ------------------------
#define AUTOADDR    1   // Automatische Addresserkennung nach Erstinitiierung oder wonn PoM-Schalter aktiv
#define ROCOADDR    2   // 0: Outputadresse 4 ist Weichenadress 1
                        // 1: Outputadresse 0 ist Weichenadress 1
const byte iniMode          = 0x50 | AUTOADDR;  // default-Betriebsmodus
const byte DccAddr          =  1;    // DCC-Decoderadresse
const byte PomAddr          = 50;    // Adresse für die Pom-Programmierung
// Standardwerte für Servoausgang
const byte iniServoGerade   = 0;     // = Par1;
const byte iniServoAbzw     = 180;   // = Par2;
const byte inispeed = 8;             // = Par3;
#define SAUTOOFF 0x01
const byte iniAutoOff       = SAUTOOFF;     // = (Mode) automatische Pulsabschaltung
// Standardwerte für Puls-Ausgang (Doppelspule)
const byte iniPulseOn       = 50;    // = (Par1) 500ms Impuls
const byte iniPulseOff      = 20;    // = (Par2) mindestens 2Sec Pause zwischen 2 Pulsen
#define CAUTOOFF 0x01
const byte iniPulseAuto     =  CAUTOOFF;    // = (Mode) automatische Pulsbegrenzung eingeschaltet

//^^^^^^^^^^^^^^^^^^ Ende der anpassbaren Konstantenwerte ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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

// allgemeine defines ---------------------------------------------------------
#define servoPins out1Pins  // Portnamen fürs Servofunktion
#define relaisPins out2Pins
#define coil1Pins out1Pins  // Portnamen für Magnetartikelfunktion
#define coil2Pins out2Pins


//-------------------------------Definition der CV-Adressen ---------------------------------------
#define CV_START    47  // Startadresse des Decoderspezifischen CV-Blocks
typedef struct {        // Definition der Struktur des decoderspezifischen CV-Blocks
    byte modeVal;       // =0x5? wenn die CV-Werte initiiert sind. Bits 0..3 für Betriebsarten
    byte PomAddrLow;      // Adresse für die POM-Programmierung
    byte PomAddrHigh;
    struct {
        byte Mode;     // Funktionalität des Ausgangs
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


// ----------------------- Variable ---------------------------------------------------
byte opMode;                    // Bit 0..3 aus modeVal
byte rocoOffs;                  // 0 bei ROCO-Adressierung, 4 sonst
byte isOutputAddr;              // Flag ob Output-Adressing
word weichenAddr;               // Addresse der 1. Weiche (des gesamten Blocks)
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
#define ADDRMODE    1   // Warte auf 1. Telegramm zur Bestimmung der ersten Weichenadresse
#define PROGMODE    2   // Adresse empfangen, POM-Programmierung möglich

// -------- Encoderauswertung ----- Justierung der Servoendlage -----------------------------
byte encoderState;
int8_t encoderCount;
#define IDLE    0   // Ruhezustand (keine Bewegung)
#define UPCOUNT 1
#define DOWNCOUNT 2
#define ACTIVE  3   // 
// Die zuletzt empfangene Weichenposition kann per Encoder justiert werden. 
// Die Werte werden gespeichert, sobald eine ander Weichenposition empfangen wird.

byte adjWix;       // Weichenindex, der z.Z. vom Encoder beeinflusst wird.
byte adjPulse;      // per Encoder aktuell eingestellte Servoposition
#define NO_ADJ 255    // Wert von adjPulse solange keine Änderung erfolgt ist

//---- Library-Objekte ----
Servo8 weicheS[WeichenZahl];
EggTimer AckImpuls;
EggTimer ledTimer;  // zum Blinken der Programmierled
// Pulsetimer für Doppelspulenantriebe
EggTimer pulseOffT[WeichenZahl];
EggTimer pulseOnT[WeichenZahl];
NmraDcc Dcc;


// für debugging ------------------------------------------------------------
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
    // Betriebsart auslesen
    if ( (Dcc.getCV( (int) &CV->modeVal)&0xf0) != ( iniMode&0xf0 ) ) {
        // In modeVal steht kein sinnvoller Wert,
        // alles initiieren mit den Defaultwerten
        // Wird über DCC ein 'factory-Reset' empfangen wird dieser Wert zurückgesetzt, was beim nächsten
        // Start ebenfalls zum initiieren führt.
        //
        // Standard-CV's
        for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
                Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
        }
        // Decoderspezifische CV's
        Dcc.setCV( (int) &CV->PomAddrLow, PomAddr%256 );
        Dcc.setCV( (int) &CV->PomAddrHigh, PomAddr/256 );
        for ( byte i = 0; i<WeichenZahl; i++ ) {
            switch ( iniTyp[i] ) {
              case FSERVO :
                Dcc.setCV( (int) &CV->Fkt[i].Mode, iniAutoOff );
                Dcc.setCV( (int) &CV->Fkt[i].Par1, iniServoGerade );
                Dcc.setCV( (int) &CV->Fkt[i].Par2, iniServoAbzw );
                Dcc.setCV( (int) &CV->Fkt[i].Par3, inispeed );
                Dcc.setCV( (int) &CV->Fkt[i].State, 0 );
                break;
              case FCOIL:
                Dcc.setCV( (int) &CV->Fkt[i].Mode, iniPulseAuto );
                Dcc.setCV( (int) &CV->Fkt[i].Par1, iniPulseOn );
                Dcc.setCV( (int) &CV->Fkt[i].Par2, iniPulseOff );
                Dcc.setCV( (int) &CV->Fkt[i].Par3, 0 );
                Dcc.setCV( (int) &CV->Fkt[i].State, 0 );
                break;
            }
        }
        Dcc.setCV( (int) &CV->modeVal, iniMode );
    }
    // Betriebsart auslesen
    opMode = Dcc.getCV( (int) &CV->modeVal) &0x0f;
    rocoOffs = ( opMode & ROCOADDR ) ? 4 : 0;

    #if  VAR_ALLWAYSPOM != 0 
        // Pom-Modus ist immer aktiv, Programmierschalter und LED werden nicht genutzt
        progMode=PROGMODE;
    #else
        // PoM-Mode nur bei aktiviertem Programmierschalter
        pinMode( A4,OUTPUT);
        pinMode( modePin, OUTPUT );
        digitalWrite( modePin, LOW );
        // Auf Programmmiermodus prüfen
        pinMode ( progPin, INPUT_PULLUP);
        if ( digitalRead( progPin) == LOW ) {
            if ( opMode & AUTOADDR ) {
                progMode= ADDRMODE; // mit automatischer Adresserkennung
            } else {
                progMode = PROGMODE; // ohne automatische Adresserkennung
            }
        } else
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
        Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_ID, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) 0) );
        CLR_PROGLED;
    } else {
        Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_ID, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) &CV->PomAddrLow) );
        #if  VAR_ALLWAYSPOM == 0 
        SET_PROGLED;
        #endif
    }

    // Encoder-Init
    IniEncoder();

    //--- Ende Grundinitiierung ---------------------------------

    setWeichenAddr();
        
    DBprintCV(); // im Debug-Mode alle CV-Werte ausgeben
    
    for ( byte i=0; i<WeichenZahl; i++ ) {
        // Funktionen initiieren
        weicheIst[i] = Dcc.getCV( (int) &CV->Fkt[i].State );
        weicheSoll[i] = weicheIst[i];
        switch (iniTyp[i] )  {
          case FSERVO:
            weicheS[i].attach( servoPins[i], Dcc.getCV(  (int) &CV->Fkt[i].Mode) & SAUTOOFF );
            weicheS[i].setSpeed( Dcc.getCV(  (int) &CV->Fkt[i].Par3 ) );
            pinMode( relaisPins[i], OUTPUT );
            // Servowerte und Relaisausgang initiieren und ausgeben
            if ( weicheSoll[i] == GERADE ) {
                weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par1 ) );
            } else {
                weicheS[i].write( Dcc.getCV( (int) &CV->Fkt[i].Par2 ) );
            }
            weicheIst[i] = weicheSoll[i];
            relaisOut[i] = weicheIst[i];
            digitalWrite( relaisPins[i], relaisOut[i] );
            break;
          case FCOIL:
            pinMode( coil1Pins[i], OUTPUT );
            pinMode( coil2Pins[i], OUTPUT );
            digitalWrite( coil1Pins[i], LOW );
            digitalWrite( coil2Pins[i], LOW );
            break;
            
        }
    }
}
////////////////////////////////////////////////////////////////
void loop() {
    if (digitalRead( A4)) digitalWrite(A4,LOW); else digitalWrite(A4,HIGH); // Test Zykluszeit
    getEncoder();    // Drehencoder auswerten und Servolage gegebenenfalls anpassen
    
    Dcc.process(); // Hier werden die empfangenen Telegramme analysiert und der Sollwert gesetzt
    
    #ifdef DEBUG
    // Merker CV für CV-Ausgabe rücksetzen (MerkerCV ist 1.CV hinter dem CV-Block für die ausgangskonfiguration)
    Dcc.setCV( (int) &CV->Fkt[WeichenZahl].Mode , 0xff );
    #endif
    
    // Ausgänge ansteuern
    for ( byte i=0; i<WeichenZahl; i++ ) {
        switch ( iniTyp[i]  ) {
          case FSERVO: // Servoausgänge ansteuern ----------------------------------------------
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
            // Relaisausgänge setzen
            digitalWrite( relaisPins[i], relaisOut[i] );
            break;

          case FCOIL: //Doppelspulenantriebe ------------------------------------------------------
            if ( ! pulseOffT[i].running() && ! (weicheIst[i]&MOVING) ) {
                // Aktionen am Ausgang nur wenn kein aktiver Impuls und der Pausentimer nicht läuft
                if ( weicheIst[i] != weicheSoll[i] ) {
                    // Weiche soll geschaltet werden
                    //DB_PRINT(" i=%d, Ist=%d, Soll=%d", i, weicheIst[i], weicheSoll[i] );
                    if ( weicheIst[i] ) {
                        // Out1 aktiv setzen
                        digitalWrite( coil1Pins[i], HIGH );
                        digitalWrite( coil2Pins[i], LOW );
                        //DB_PRINT( "Pin%d HIGH, Pin%d LOW", coil1Pins[i], coil2Pins[i] );
                    } else {
                        // Out2 aktiv setzen
                        digitalWrite( coil2Pins[i], HIGH );
                        digitalWrite( coil1Pins[i], LOW );
                        //DB_PRINT( "Pin%d LOW, Pin%d HIGH", coil1Pins[i], coil2Pins[i] );
                    }
                    pulseOnT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par1 ) * 10 );
                    weicheIst[i] = weicheSoll[i] | MOVING;
                    Dcc.setCV( (int) &CV->Fkt[i].State, weicheSoll[i] );
                }
                
            }
            // Timer für Spulenantriebe abfragen
            if ( iniTyp[i] == FCOIL ) {
                // prüfen ab Impuls abgeschaltet werden muss
                // (Timer läuft nicht mehr, aber MOVING-Bit noch gesetzt)
                if ( !pulseOnT[i].running() && (weicheIst[i]&MOVING) ) {
                    digitalWrite( coil2Pins[i], LOW );
                    digitalWrite( coil1Pins[i], LOW );
                    weicheIst[i]&= ~MOVING;
                    //DB_PRINT( "Pin%d LOW, Pin%d LOW", coil1Pins[i], coil2Pins[i] );
                    pulseOffT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par2 ) * 100 );
                }
                
            }
            break;
        } // - Ende Switch Funktionstypen-------------------------------------------
    } // Ende Schleife über die Funktionen (Weichen)---------------



    // Ackimpuls abschalten--------------------------
    if ( !AckImpuls.running() ) digitalWrite( ackPin, LOW );

    #if VAR_ALLWAYSPOM == 0
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
    byte i;
    word wAddr = Addr+rocoOffs; // Roco zählt ab 0, alle anderen lassen die ersten 4 Weichenadressen frei
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
            weichenAddr = (Dcc.getAddr( )-1)*4 +1 + rocoOffs;
        }
        progMode = PROGMODE;
        DB_PRINT( "Neu: Boardaddr: %d, 1.Weichenaddr: %d", BoardAddr, weichenAddr );
    }
    // Testen ob eigene Weichenadresse
    //DB_PRINT( "DecAddr=%d, Weichenadresse: %d , Ausgang: %d, State: %d", BoardAddr, wAddr, OutputAddr, State );
    for ( i = 0; i < WeichenZahl; i++ ) {
        if (  wAddr == weichenAddr+i ) {
            // ist eigene Adresse, Sollwert setzen
            weicheSoll[i] =  OutputAddr & 0x1;
            DB_PRINT( "Weiche %d, Index %d, Soll %d, Ist %d", wAddr, i, weicheSoll[i],  weicheIst[i] );
            break; // Schleifendurchlauf abbrechen, es kann nur eine Weiche sein
        }
    }
    ChkAdjEncode( i );
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
        switch ( Dcc.getCV( (uint16_t) &CV->Fkt[i].Mode ) ) {
          case FSERVO:
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
        if ( CvAddr ==  (int) &CV->Fkt[WeichenZahl].Mode && Value !=0xff ) {
            DBprintCV();
        }
    #endif

    // prüfen ob die Weichenadresse verändert wurde. Dies kann durch Ändern der
    // Adressierungsart in CV29 oder direkt durch Ändern der Decoderadresse geschehen.
    // Wird die Decoderadresse geändert muss zuerst das MSB (CV9) verändert werden. Mit dem
    // Ändern des LSB (CV1) wird dann die Weichenadresse neu berechnet
    if ( CvAddr ==  29 || CvAddr ==  1 ) setWeichenAddr();

    // Prüfen ob die Betriebsart des Decoders verändert wurde
    if ( CvAddr == (int) &CV->modeVal ) {
        // Die Betriebsart wurde verändert -> Neustart
        delay( 500 );
        softReset();
    }
}    
//-----------------------------------------------------
void notifyCVResetFactoryDefault(void) {
    // Auf Standardwerte zurücksetzen und Neustart
    Dcc.setCV( (int) &CV->modeVal, 255 );
    delay( 500 );
    softReset();
}
//------------------------------------------------------
#ifdef DEBUG
void notifyDccReset( uint8_t hardReset ) {
    if ( hardReset > 0 ) DB_PRINT("Reset empfangen, Value: %d", hardReset);
    // wird bei CV-Auslesen gesendet
}
#endif
//--------------------------------------------------------

/////////////////////////////////////////////////////////////////////////
// Unterprogramme zur Servojustierung mit Drehencoder
void IniEncoder( void ) {
    // Encoder initiieren
    pinMode( encode1P, INPUT_PULLUP );
    pinMode( encode2P, INPUT_PULLUP );
    encoderState = IDLE;
    encoderCount  = 0;
    adjWix = WeichenZahl;
    adjPulse = NO_ADJ;
}
   
void getEncoder( void ) {
    // Encoder-Statemachine
    switch ( encoderState ) {
      case IDLE: // Grundstellung, beide Eingänge sind high
        if ( digitalRead( encode1P ) == 0 ) encoderState = UPCOUNT;
        if ( digitalRead( encode2P ) == 0 ) encoderState = DOWNCOUNT;
        break;
      case UPCOUNT:
        if ( digitalRead( encode2P )== 0 ) {
            // beide Eingänge aktiv, hochzählen
            encoderCount++;
            encoderState = ACTIVE;
        } else if ( digitalRead( encode1P ) ) {
            encoderState = IDLE;
        }
        break;
      case DOWNCOUNT:
        if ( digitalRead( encode1P )== 0 ) {
            // beide Eingänge aktiv, runterzählen
            encoderCount--;
            encoderState = ACTIVE;
        } else if ( digitalRead( encode2P ) ) {
            encoderState = IDLE;
        }
        break;
      case ACTIVE: // Warten bis Ruhelage
        if ( digitalRead( encode2P ) && digitalRead( encode1P ) ) encoderState = IDLE;
        break;
    }
    #ifdef DEBUG
    // encoderzähler ausgeben
    if ( encoderCount != 0 ) {
        DB_PRINT( "Encoder: %d", encoderCount );        
    }
    #endif

    if ( encoderCount != 0 && adjWix < WeichenZahl && !(weicheIst[adjWix] & MOVING ) ) {
        // Drehencoder wurde bewegt, und es gibt eine aktuell zu justierende Weiche, die sich nicht
        // gerade bewegt
        if ( adjPulse == NO_ADJ ) {
            // ist erster Justierimpuls, aktuelle Position aus CV auslesen
            if ( weicheSoll[adjWix] == GERADE ) {
                adjPulse = Dcc.getCV( (int) &CV->Fkt[adjWix].Par1 );
            } else {
                adjPulse = Dcc.getCV( (int) &CV->Fkt[adjWix].Par2 );
            }
        }
        if ( (encoderCount>0 && adjPulse<180) || (encoderCount<0 && adjPulse>0) )
            adjPulse += encoderCount; // adjPulse nur im Bereich 0...180 
        weicheS[adjWix].write( adjPulse );
    }
    encoderCount = 0;
}

void ChkAdjEncode( byte WIndex ){
    // nach dem Empfang einer Weichenadresse wird geprüft, ob eine vorherige Justierung gespeichert werden
    // muss. Die empfangene Weichenadresse wird als neue Justieradresse gespeichert wenn es sich um einen
    // Servoantrieb handelt.
    if ( adjPulse != NO_ADJ ) {
        // Es wurde justiert, testen ob gespeichert werden muss (Weichenwechsel)
        if ( WIndex != adjWix || weicheIst[adjWix] != weicheSoll[adjWix] ) {
            // Weiche wurde umgeschaltet, oder eine andere Weiche betätigt -> Justierung speichern
            if ( weicheIst[adjWix] == GERADE ) {
                Dcc.setCV( (int) &CV->Fkt[adjWix].Par1, adjPulse );
            } else {
                Dcc.setCV( (int) &CV->Fkt[adjWix].Par2, adjPulse );
            }
            adjPulse = NO_ADJ;
        }
    }
    adjWix = WIndex;
    if ( adjWix < WeichenZahl ) 
        if ( iniTyp[ adjWix ] != FSERVO ) adjWix = WeichenZahl;
    
}




//////////////////////////////////////////////////////////////////////////
// Allgemeine Unterprogramme 
void setWeichenAddr(void) {
    // Adressmodus aus CV29 auslesen
    isOutputAddr = Dcc.getCV( CV_29_CONFIG ) & CV29_OUTPUT_ADDRESS_MODE;
    // Adresse der 1. Weiche aus Decoderaddresse berechnen
    if ( isOutputAddr ) 
        weichenAddr = Dcc.getAddr( );
    else
        weichenAddr = (Dcc.getAddr( )-1)*4 +1 + rocoOffs ;
}
//--------------------------------------------------------
void softReset(void){
;asm volatile ("  jmp 0");
}
//-----------------------------------------------------------
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
    DB_PRINT ( "Initierungswert: 0x%x (%d) ", Dcc.getCV( (int) &CV->modeVal ), Dcc.getCV( (int) &CV->modeVal ) );
    DB_PRINT ( "PoM-Adresse    : %d"   , Dcc.getCV( (int) &CV->PomAddrLow) + 256* Dcc.getCV( (int) &CV->PomAddrHigh ) );
    
    // Output-Konfiguration
    DB_PRINT( "Wadr | Typ | CV's  | Mode | Par1 | Par2 | Par3 | Status |");
    for( byte i=0; i<WeichenZahl; i++ ) {
        DB_PRINT( "%4d |%4d | %2d-%2d | %4d | %4d | %4d | %4d | %3d ", weichenAddr+i, iniTyp[i],
                                                                 &CV->Fkt[i].Mode,  &CV->Fkt[i].State,
                                                                 Dcc.getCV( (int)  &CV->Fkt[i].Mode ),
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


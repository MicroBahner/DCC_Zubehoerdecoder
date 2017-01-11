#include <NmraDcc.h>
#include <MobaTools.h>

/* Weichendecoder für Greißelbach
 *   Version 0.1 - erstmal nur Servos
 *   Version 0.2 - alternativ auch ohne Programmierschalter nutzbar. PoM ist dann immer aktiv,
 *                  Addressen nur über den Sketch änderbar.
 *                  Adressierung als Board- oder Outputadressierung je nach CV29:6 (1=Outputaddr.)
 *                  Ansteuerung von Doppelspulenantrieben 
 *   Version 2.1   Einstellen der Servoendlagen per Drehencoder. Wegen der 2 Encodereingänge
 *                  können maximal 6 Weichen angesteuert werden.
 *                  Der Drehencoder bezieht sich immer auf die zuletzt gestellte Weiche.
 *   Version 3.0    Die Betriebsmodi und Startverhalten wird jetzt über die Analogeingänge A4/A5 eingestellt. Dazu 
 *                  müssen dort Pullups eingebaut werden. Jenachdem wieweit die Spannung  heruntergezogen wird werden
 *                  die Modi eingestellt:
 *                  A5:
 *                  5V (nur Pullup) normaler Betriebsmodus, kein PoM
 *                  3,3V (Spannungsteiler 1:2) PoM immer aktiv, Adresse immer aus defaults
 *                  1,6V (Spannungsteiler 2:1) IniMode: CV's werden immer auf init-Werte aus .h-Datei gesetzt
 *                  0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *                  A4:
 *                  wird A4 auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
 *                  Mittellage gebracht. Sobald der Encoder wieder bewegt wird, bewegt sich das Servo wieder
 *                  zur vorhergehenden Position.
 *                  Ist A4 beim Programmstart auf 0, werden alle CV's auf die Defaults zurückgesetzt
 *                  Bei Nano- und Mini-Versionen kann dies auf A6/A7 umgestellt werden, um Ports freizumachen
 *                  (A6/7 sind beim UNO nicht vorhanden)
 *                  
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
#define DCC_DECODER_VERSION_ID 0x30
// für debugging ------------------------------------------------------------
//#define DEBUG ;             // Wenn dieser Wert gesetzt ist, werden Debug Ausgaben auf dem ser. Monitor ausgegeben

#ifdef DEBUG
#define DB_PRINT( x, ... ) { sprintf_P( dbgbuf, PSTR( x ), __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
static char dbgbuf[80];
#else
#define DB_PRINT ;
#endif


//------------------------------------------ //
// die NmraDcc - Library gibt es unter https://github.com/mrrwa/NmraDcc/archive/master.zip

#define NC 0xff    // nicht verwendeten Funktionsausgängen kann der Port NC zugeweisen werden.
// die arduino digitalWrite und digitalRead Funktionen prüfen auf gültige Pin-Nummern und machen nichts
// bei ungültigen Nummern. 0xff ist nie eine gültige Pinnummer


// Grenzwerte des Analogeingangs für die jeweiligen Betriebsmodi ( gesamter Bereich 0...1024):
#define ISNORMAL    853         // > 853 gilt als normalbetrieb
#define ISPOM       512         // <853,  >512 Allway Pom
#define ISOPEN      170         // <512, >170 IniMode: alle Funktionsparameter beim Start initiieren
#define ISPROG      0           // <170 Programmiermodus (Adresserkennung)

// Mögliche Funktionstypen je Ausgang. 
#define FOFF        0 // Funktionsausgang abgeschaltet
#define FSERVO      1 // Standardservoausgang 
#define FCOIL       2 // Magnetartikel
#define FSTATIC     3 // Der Ausgang wird statisch/blinkend ein bzw ausgeschaltet
#define FMAX        3  

//---------------------------------------
#define AUTOADDR    1   // Automatische Addresserkennung nach Erstinitiierung oder wenn Programmiermodus aktiv
#define ROCOADDR    2   // 0: Outputadresse 4 ist Weichenadress 1
                        // 1: Outputadresse 0 ist Weichenadress 1
#define SAUTOOFF 0x01
#define CAUTOOFF 0x01
#define BLKMODE 0x01    // FSTATIC: Ausgänge blinken
#define BLKSTRT 0x02    // FSTATIC: Starten mit beide Ausgängen EIN

const byte dccPin       =   2;
const byte ackPin       =   4;
#include "DCC_Zubehoerdecoder.h"

const byte WeichenZahl = sizeof(iniTyp);

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
        byte Mode;    // Funktionalität des Ausgangs
        byte Par1;    // Servo: Position AUS ( 0...180 Grad )
                      // DoppelspuleAuto: Einschaltzeit ( 10ms-Schritte )
                      // FSTATIC:        Einschaltphase des Blinkens ( 10ms Schritte )
        byte Par2;    // Servo: Position EIN ( 0...180 Grad )
                      // DoppelspuleAuto: Einschaltzeit Ausgang 2 ( 10ms-Schritte ) 255=Dauerein
                      // FSTATIC:        Ausschaltphase des Blinkens ( 10ms Schritte )
        byte Par3;    // Servo: Speed
        byte State;   // aktueller Status des Ausgangs (Ein/Aus)
                      // Nach einem Neustart werden die Funktionsausgänge entsprechend eingestellt.
    } Fkt [WeichenZahl];
} CvVar_t;

const CvVar_t *CV = (CvVar_t *) CV_START; //Pointer auf die decoderspezifischen CV-Werte im EEProm
#define GetCvPar(ix,par) Dcc.getCV((int)&CV->Fkt[ix].par)

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
#define BLKON   0x4               // nur für Ist-Zustand, Blinken ist EIN

//int geradePulse[WeichenZahl] ;  // Pulslänge geradeaus
//int abzweigPulse[WeichenZahl];  // Pulslänge abzweigend

byte relaisOut[WeichenZahl];    // Ausgabewerte für die Relais ( 0/1, entspricht Sollwert )
byte pulseON[WeichenZahl];    // CoiL: Flag ob Pausentimer läuft

byte progMode;      // Merker ob Decoder im Programmiermodus
#define NORMALMODE  0
#define ADDRMODE    1   // Warte auf 1. Telegramm zur Bestimmung der ersten Weichenadresse ( Prog-Led blinkt )
#define PROGMODE    2   // Adresse empfangen, POM-Programmierung möglich ( Prog-Led aktiv )
#define POMMODE     3   // Allways-Pom Mode ( keine Prog-Led )
#define INIMODE     4   // Wie Normalmode, aber die StandardCV-Werte und CV47-49 werden bei jedem
                        // Start aus den defaults geladen

#define SET_PROGLED digitalWrite( modePin, HIGH )
#define CLR_PROGLED digitalWrite( modePin, LOW )

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
// Pulsetimer für Doppelspulenantriebe und Blinken
EggTimer pulseT[WeichenZahl];
EggTimer debounceT;   // Encoder entprellen
const byte debTime = 4;
NmraDcc Dcc;


//^^^^^^^^^^^^^^^^^^^^^^^^ Ende der Definitionen ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//###########################################################################


void setup() {
    // Betriebsart auslesen
    int temp = analogRead( betrModeP );
    if ( temp > ISNORMAL ) {
        // Normalbetrieb
        progMode = NORMALMODE;
    } else if ( temp > ISPOM ) {
        // PoM immer aktiv 
        progMode = POMMODE;
    } else if ( temp > ISOPEN ) {
        // IniMode - GrundCV's werden immer initiiert
        progMode = INIMODE;
    } else {
        // Programmiermodus, automatische Adresserkennung
        progMode = ADDRMODE;
    }
    

    
    if ( (Dcc.getCV( (int) &CV->modeVal)&0xf0) != ( iniMode&0xf0 ) || analogRead(resModeP) < 100 ) {
        // In modeVal steht kein sinnvoller Wert ( oder resModeP ist auf 0 ),
        // alles initiieren mit den Defaultwerten
        // Wird über DCC ein 'factory-Reset' empfangen wird modeVal zurückgesetzt, was beim nächsten
        // Start zum initiieren führt.
        //
        // Standard-CV's
        for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
                Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
        }
        // Decoderspezifische CV's
        Dcc.setCV( (int) &CV->PomAddrLow, PomAddr%256 );
        Dcc.setCV( (int) &CV->PomAddrHigh, PomAddr/256 );
        Dcc.setCV( (int) &CV->modeVal, iniMode );
        for ( byte i = 0; i<WeichenZahl; i++ ) {
            Dcc.setCV( (int) &CV->Fkt[i].Mode, iniFmode[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par1, iniPar1[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par2, iniPar2[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par3, iniPar3[i] );
            Dcc.setCV( (int) &CV->Fkt[i].State, 0 );
        }
    } else if ( progMode == INIMODE ) {
        // Standard-CV's immer initiieren
        for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
                Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
        }
        Dcc.setCV( (int) &CV->PomAddrLow, PomAddr%256 );
        Dcc.setCV( (int) &CV->PomAddrHigh, PomAddr/256 );
        Dcc.setCV( (int) &CV->modeVal, iniMode );
        
        // Funktionsspezifische Parameter aus INI-Tabelle laden
        for ( byte i = 0; i<WeichenZahl; i++ ) {
            Dcc.setCV( (int) &CV->Fkt[i].Mode, iniFmode[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par1, iniPar1[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par2, iniPar2[i] );
            Dcc.setCV( (int) &CV->Fkt[i].Par3, iniPar3[i] );
        }
        
    }
    
    // Betriebsart auslesen
    opMode = Dcc.getCV( (int) &CV->modeVal) &0x0f;
    rocoOffs = ( opMode & ROCOADDR ) ? 4 : 0;
    
    #ifdef DEBUG
      Serial.begin(115200); //Debugging
      DB_PRINT( "Betr:%d -> Mode=", temp );
      switch ( progMode ) {
        case NORMALMODE:
          Serial.println( "Std" );
          break;
        case ADDRMODE:
          Serial.println( "Addr" );
          break;
        case POMMODE:
          Serial.println( "Pom" );
          break;
        case INIMODE:
          Serial.println( "Ini" );
          break;
        default:
          Serial.println( "??" );
          
      }
    #endif
    
    pinMode( ackPin, OUTPUT );
    Dcc.pin( digitalPinToInterrupt(dccPin), dccPin, 1); 
    if ( progMode == NORMALMODE ) {
        Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_ID, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) 0) );
        CLR_PROGLED;
    } else {
        Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_ID, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) &CV->PomAddrLow) );
        SET_PROGLED;
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
            pulseON[i] = false;
            digitalWrite( coil1Pins[i], LOW );
            digitalWrite( coil2Pins[i], LOW );
            break;
          case FSTATIC:
            pinMode( out1Pins[i], OUTPUT );
            pinMode( out2Pins[i], OUTPUT );

            if ( GetCvPar(i,Mode) & BLKMODE ) {
                // aktuellen Blinkstatus berücksichtigen
                digitalWrite( out1Pins[i], LOW );
                digitalWrite( out2Pins[i], LOW );
            } else {
                // statische Ausgabe
                digitalWrite( out1Pins[i], weicheIst[i] & 0x1 );
                digitalWrite( out2Pins[i], !(weicheIst[i] & 0x1  ) );
            }
            break;
            
        }
    }
}
////////////////////////////////////////////////////////////////
void loop() {
    //if (digitalRead( A4)) digitalWrite(A4,LOW); else digitalWrite(A4,HIGH); // Test Zykluszeit
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
                //DB_PRINT( "WeicheIx=%d stellen, Ist=%d,Soll=%d", i, weicheIst[i], weicheSoll[i] );
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
            if ( ! ( !pulseON[i] && pulseT[i].running() ) && ! (weicheIst[i]&MOVING) ) {
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
                    pulseON[i] = true;
                    pulseT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par1 ) * 10 );
                    weicheIst[i] = weicheSoll[i] | MOVING;
                    Dcc.setCV( (int) &CV->Fkt[i].State, weicheSoll[i] );
                }
                
            }
            // Timer für Spulenantriebe abfragen
            if ( pulseON[i] ) {
                // prüfen ab Impuls abgeschaltet werden muss
                // (Timer läuft nicht mehr, aber MOVING-Bit noch gesetzt)
                if ( !pulseT[i].running() && (weicheIst[i]&MOVING) ) {
                    digitalWrite( coil2Pins[i], LOW );
                    digitalWrite( coil1Pins[i], LOW );
                    weicheIst[i]&= ~MOVING;
                    //DB_PRINT( "Pin%d LOW, Pin%d LOW", coil1Pins[i], coil2Pins[i] );
                    pulseON[i] = false;
                    pulseT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par2 ) * 10 );
                }
                
            }
            break;
          case FSTATIC: // Ausgang statisch ein/ausschalten ------------------------------------
            // muss Ausgang umgeschaltet werden?
            if ( (weicheSoll[i]&1) != (weicheIst[i]&1) ) {
                digitalWrite( out1Pins[i], weicheSoll[i] );
                if ( GetCvPar(1,Mode) & BLKMODE ) {
                    digitalWrite( out2Pins[i], (GetCvPar(i,Mode) & BLKSTRT)&& (weicheSoll[i]&1) ); 
                } else {                   
                    digitalWrite( out2Pins[i], !weicheSoll[i] );                    
                }
                DB_PRINT( "Soll=%d, Ist=%d", weicheSoll[i], weicheIst[i] );
                weicheIst[i] = weicheSoll[i];
                Dcc.setCV( (int) &CV->Fkt[i].State, weicheIst[i] );
                if ( weicheIst[i] && ( Dcc.getCV( (int) &CV->Fkt[i].Mode ) & BLKMODE ) ) {
                    // Funktion wird eingeschaltet und Blinkmode ist aktiv -> Timer setzen
                    pulseT[i].setTime( GetCvPar(i,Par3)*10 );
                    DB_PRINT( "BlkEin %d/%d, Strt=%x", GetCvPar(i,Par1) , GetCvPar(i,Par2), (GetCvPar(i,Mode) & BLKSTRT)  );
                    weicheIst[i] |= BLKON;
                }
            }
            if ( weicheIst[i] && ( Dcc.getCV( (int) &CV->Fkt[i].Mode ) & BLKMODE ) ) {
                // bei aktivem Blinken die Timer abfragen/setzen
                if ( !pulseT[i].running() ) {
                    // Timer abgelaufen, Led-Status wechseln
                    if ( weicheIst[i] & BLKON ) {
                        // Led ausschalten
                        digitalWrite( out1Pins[i], LOW );
                        digitalWrite( out2Pins[i], HIGH );
                        weicheIst[i] &= ~BLKON;
                        pulseT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par2 )*10 );
                    } else {
                        // Led einschalten
                        digitalWrite( out1Pins[i], HIGH );
                        digitalWrite( out2Pins[i], LOW );
                        weicheIst[i] |= BLKON;
                        pulseT[i].setTime( Dcc.getCV( (int) &CV->Fkt[i].Par1 )*10 );
                    }
                    
                }
                
            }
            

            break;
        } // - Ende Switch Funktionstypen-------------------------------------------
    } // Ende Schleife über die Funktionen (Weichen)---------------



    // Ackimpuls abschalten--------------------------
    if ( !AckImpuls.running() ) digitalWrite( ackPin, LOW );

    // Programmierled blinkt im Programmiermode bis zum Empfang einer Adresse
    if ( ! ledTimer.running() && progMode == ADDRMODE) {
        ledTimer.setTime( 500 );
        if ( digitalRead( modePin ) ) 
            CLR_PROGLED;
        else
            SET_PROGLED;
    }
}
//////////////////////////////////////////////////////////////
// Unterprogramme, die von der DCC Library aufgerufen werden:
//------------------------------------------------
// Die folgende Funktion wird von Dcc.process() aufgerufen, wenn ein Weichentelegramm empfangen wurde
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State ){
    // Weichenadresse berechnen
    byte i;
    word wAddr = Addr+rocoOffs; // Roco zählt ab 0, alle anderen lassen die ersten 4 Weichenadressen frei
    // Im Programmiermodus bestimmt das erste empfangene Programm die erste Weichenadresse
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
        SET_PROGLED;
        //DB_PRINT( "Neu: Boardaddr: %d, 1.Weichenaddr: %d", BoardAddr, weichenAddr );
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
        switch ( iniTyp[i] ) {
          case FSERVO:
            // gehört der veränderte CV zu diesem Servo?
            if (  (CvAddr == (uint16_t) &CV->Fkt[i].Par1 && weicheSoll[i] == GERADE) ||
                  (CvAddr == (uint16_t) &CV->Fkt[i].Par2 && weicheSoll[i] == ABZW ) ){
                // Es handelt sich um die aktuelle Position des Servos,
                // Servo neu positionieren
                //DB_PRINT( "Ausg.%d , Pos. %d neu einstellen", i, weicheSoll[i] );
                 weicheS[i].write( Value );
            } else if ( CvAddr == (uint16_t) &CV->Fkt[i].Par3 ) {
                // die Geschwindigkeit des Servo wurde verändert
                //DB_PRINT( "Ausg.%d , Speed. %d neu einstellen", i, Value );
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
    //if ( hardReset > 0 ) DB_PRINT("Reset empfangen, Value: %d", hardReset);
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
    if ( debounceT.running() ) return; // keine Abfrage während Entprellzeit läuft
    debounceT.setTime( debTime );
    switch ( encoderState ) {
      case IDLE: // Grundstellung, beide Eingänge sind high
        if ( digitalRead( encode1P ) == 0 ) encoderState = UPCOUNT;
        if ( digitalRead( encode2P ) == 0 ) encoderState = DOWNCOUNT;
        break;
      case UPCOUNT:
        if ( digitalRead( encode2P )== 0 && digitalRead( encode1P )== 0  ) {
            // beide Eingänge aktiv, hochzählen
            encoderCount++;
            encoderState = ACTIVE;
        } else if ( digitalRead( encode1P ) && digitalRead( encode2P ) ) {
            #ifdef ENCODER_DOUBLE
            encoderCount++;
            #endif
            encoderState = IDLE;
        }
        break;
      case DOWNCOUNT:
        if ( digitalRead( encode1P )== 0  && digitalRead( encode2P )== 0 ) {
            // beide Eingänge aktiv, runterzählen
            encoderCount--;
            encoderState = ACTIVE;
        } else if ( digitalRead( encode2P ) && digitalRead( encode1P ) ) {
            #ifdef ENCODER_DOUBLE
            encoderCount--;
            #endif
            encoderState = IDLE;
        }
        break;
      case ACTIVE: // Warten bis Ruhelage
        if ( digitalRead( encode1P ) == 1 ) encoderState = UPCOUNT;
        if ( digitalRead( encode2P ) == 1 ) encoderState = DOWNCOUNT;
        break;
    }
    #ifdef DEBUG
    // encoderzähler ausgeben
    if ( encoderCount != 0 ) {
        //DB_PRINT( "Encoder: %d", encoderCount );        
    }
    #endif

    if ( adjWix < WeichenZahl && !(weicheIst[adjWix] & MOVING ) ) {
        // es gibt eine aktuell zu justierende Weiche, die sich nicht
        // gerade bewegt
        if ( encoderCount != 0 ) {
            // Drehencoder wurde bewegt 
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
        } else if ( analogRead( resModeP ) < 500 ) {
            // Mittelstellungstaster gedrückt
            weicheS[adjWix].write( 90 );
        }
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
    DB_PRINT( "--------- Debug-Ausgabe CV-Werte ---------", 0 );
    DB_PRINT( "Version: %d, ManufactId: %d", Dcc.getCV( CV_VERSION_ID ), Dcc.getCV( CV_MANUFACTURER_ID ) );
    DB_PRINT( "Konfig   (CV29)  : 0x%X", Dcc.getCV( CV_29_CONFIG ) );
    DB_PRINT( "Adresse: (CV1/9) : %d", Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_LSB )+Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_MSB )*256);
    DB_PRINT( "1.Weichenaddresse: %d", weichenAddr );
    
    // Decoder-Konfiguration global
    DB_PRINT( "Initierungswert: 0x%x (%d) ", Dcc.getCV( (int) &CV->modeVal ), Dcc.getCV( (int) &CV->modeVal ) );
    DB_PRINT( "PoM-Adresse    : %d"   , Dcc.getCV( (int) &CV->PomAddrLow) + 256* Dcc.getCV( (int) &CV->PomAddrHigh ) );
    
    // Output-Konfiguration
    DB_PRINT( "Wadr | Typ | CV's  | Mode | Par1 | Par2 | Par3 | Status |",0 );
    for( byte i=0; i<WeichenZahl; i++ ) {
        DB_PRINT( "%4d |%4d | %2d-%2d | %4d | %4d | %4d | %4d | %3d " , weichenAddr+i, iniTyp[i],
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


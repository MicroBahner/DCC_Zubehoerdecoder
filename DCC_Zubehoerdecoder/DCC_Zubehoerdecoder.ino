/* Universeller DCC-Decoder für Weichen und (Licht-)Signale
    Version 6.3.0 - Die Funktionalitäten sind als Klassen definiert 
    Die Klassenobjekte werden erst im Setup je nach Konfiguration instanziiert
    
   Eigenschaften:
   Mehrere (aufeinanderfolgende) Zubehöradressen ansteuerbar
   Die max. Zahl der Adressen hängt im Wesentlichen von der Zahl der Digitalausgänge ab
   (max 16 Servos sind konfigurierbar)
   1. Adresse per Programmierung einstellbar
   
   3 Ausgänge / Zubehöradresse
   Einstellbare Funktionalität:
    - Servo mit Umschaltrelais zur Weichenpolarisierung
    - logische Kopplung von 2 Servos für 3-begriffige Formsignale
    - 1 Servo über 2 Adressen um 4 Positionen anzusteuern
    - Impulsfunktion für Servos ( automatisches Rückkehren in Ausgangslage,
      z.B. für Entkuppler )
    - Doppelspulenantriebe
    - statische Ausgänge
    - blinkende Ausgänge
    - Lichtsignalfunktionen
    
    Die Funnktionalität und IO-Zuordnung wird über Tabellen im h-File festgelegt.
    Die Konfiguration der einzelnen Funktionen geschieht über CV-Programmierung.
    So sind z.B. bei Servoausgängen die Endlagen per CV-Wert einstellbar, bei Lichtsignalen ist die 
    Zuordnung der Ausgangszustände zum Signalzustand frei konfigurierbar.
*/
#define DCC_DECODER_VERSION_ID 0x63

#include "src/FuncClasses.h"
#ifdef __AVR_MEGA__
#include <avr/wdt.h>    // für Soft-Reset ( über Watchdog )
#endif

//------------------------------------------ //


#ifdef __STM32F1__
    // ist jetzt im core definiert: #define digitalPinToInterrupt(x) x
    #define MODISTEP    4096/6      // Grenzwerte am Analogeingang der Betriebsmodi
#else
    #define MODISTEP    1024/6
#endif
#define uint_t unsigned int


// Grenzwerte des Analogeingangs für die jeweiligen Betriebsmodi ( gesamter Bereich 0...1024):
#define ISNORMAL    MODISTEP*5          // > 853 gilt als normalbetrieb
#define ISPOM       MODISTEP*3          // <853,  >512 Allway Pom
#define ISOPEN      MODISTEP            // <512, >170 IniMode: alle Funktionsparameter beim Start initiieren
#define ISPROG      0                   // <170 Programmiermodus (Adresserkennung)

// Mögliche Funktionstypen je Ausgang. 
#define FOFF        0 // Funktionsausgang abgeschaltet
#define FSERVO      1 // Standardservoausgang 
#define FCOIL       2 // Magnetartikel
#define FSTATIC     3 // Der Ausgang wird statisch/blinkend ein bzw ausgeschaltet
#define FSIGNAL0    4 // Folgeadresse für Signale
#define FSIGNAL2    5 // 1. Signaladresse 
#define FVORSIG     6 // 1. Vorsignaladresse
#define FSERVO0     7 // Folgeadresse bei 2 gekoppelten Servos
#define FMAX        7  

//---------------------------------------
//Flags für iniMode:
#define AUTOADDR    1   // Automatische Addresserkennung nach Erstinitiierung oder wenn Programmiermodus aktiv
#define ROCOADDR    2   // 0: Outputadresse 4 ist Weichenadress 1
                        // 1: Outputadresse 0 ist Weichenadress 1
//-----------------------------------------
//------------------ Einbinden der Konfigurationsdatei -------------------------
#ifdef __STM32F1__
#include "DCC_Zubehoerdecoder-STM32.h"
#elif defined(__AVR_ATmega32U4__)
#include "DCC_Zubehoerdecoder-Micro.h"
#else
#include "DCC_Zubehoerdecoder.h"
//#include "TestKOnf\DCC_Zubehoerdecoder-LS-Nano.h"
//#include "examples\DCC_Zubehoerdecoder-Micro.h"
#endif
//-------------------------------------------------------------------------------
//-------------------------------------------
const byte WeichenZahl = sizeof(iniTyp);



#define cvAdr(wIx,par)      (uint16_t)CV_FUNCTION+CV_BLKLEN*(wIx)+(par)
#define getCvPar(wIx,par)   ifc_getCV( cvAdr(wIx,par) )
#define cvEAdr(wIx,epar)    CV_EXTDATA+epar+CV_ERWLEN*wIx  
#define getCvExtPar(wIx,epar) ifc_getCV( cvEAdr(wIx,epar) )

// CV Default-Werte der Standardadressen:
struct CVPair {
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  {cvAccDecAddressLow, DccAddr%256},
  {cvAccDecAddressHigh, DccAddr/256},
  {cvVersionId, DCC_DECODER_VERSION_ID},
  {cvManufactId, manIdValue},
  {cv29Config, config29Value},
};

// ----------------------- Variable ---------------------------------------------------
byte opMode;                    // Bit 0..3 aus modeVal
byte rocoOffs;                  // 4 bei ROCO-Adressierung, 0 sonst
//byte isOutputAddr=1;            // Flag ob Output-Adressing ab 6.1 immer Ouptu-Adressind
word weichenAddr;               // Addresse der 1. Weiche (des gesamten Blocks)
byte ioPins[PPWA*WeichenZahl];  // alle definierten IO's in einem linearen Array

// Pointer auf die Funktionsobjekte
union { // für jede Klasse gibt es ein Array, die aber übereinanderliegen, da pro Weichenadresse
        // nur ein Objekt möglich ist
    Fservo  *servo[WeichenZahl];
    Fcoil   *coil[WeichenZahl];    
    Fstatic *stat[WeichenZahl];   
    Fsignal *sig[WeichenZahl];
}Fptr;    

Fservo *AdjServo = NULL ;   // Pointer auf zu justierenden Servo
// Typkennung für verbundene Adressen (Adressen mit Folgeeinträgen bei Servos oder Lichtsignalen
// Diese Kennung wird immer nur bei der Grundadresse eingetragen
enum combine_t:byte { NOCOM,        // keine Folgeadresse vorhanden, Defaultwert
                      SERVO4POS,    // Servo mit 4 Positionen
                      SERVO_DOUBLE, // 2 verbundene Servos
                      SIGNAL2ADR,   // Lichtsignal mit 4 Signalbildern
                      SIGNAL3ADR }; // Lichtsignal mit 6 Signalbildern
combine_t adressTyp[WeichenZahl];
;
byte progMode;      // Merker ob Decoder im Programmiermodus
// -------- Encoderauswertung ----- Justierung der Servoendlage -----------------------------
# ifdef ENCODER_AKTIV
// Die zuletzt empfangene Weichenposition kann per Encoder justiert werden. 
// Die Werte werden gespeichert, sobald eine ander Weichenposition empfangen wird.
//byte adjWix;        // Weichenindex, der z.Z. vom Encoder beeinflusst wird.
byte adjPos;        // Position des Servo, das vom Encoder beeiflusst wird
byte adjPulse;      // per Encoder aktuell eingestellte Servoposition
#define NO_ADJ 255  // Wert von adjPulse solange keine Änderung erfolgt ist
#endif
bool localCV;       // lokale Änderung eines CV (Callback NotifyCV wird dann nicht ausgeführt )
//---- Library-Objekte ----
EggTimer AckImpuls;
EggTimer ledTimer;  // zum Blinken der Programmierled
#ifdef LOCONET
// Bei der Loconet-Schnittstelle wird 2Sec nach Ändern der 'Pom' Adresse ein Reset ausgeführt, wobei
// die Pom-Adress als Loconet ID übernommen wird. Die Zeitverzögerung ist erforderlich, damit Low- und High
// Byte geschrieben werden können, bevor der Reset ausgeführt wird. Die Zeit startet, wenn eins der beiden
// Byte geschrieben wird.
bool chgLoconetId = false;
EggTimer idLoconet;
#endif

//^^^^^^^^^^^^^^^^^^^^^^^^ Ende der Definitionen ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//###########################################################################

void setup() {

#ifdef __STM32F1__
   disableDebugPorts();     // JTAG und SW Ports freigeben
#endif
    #ifdef FIXMODE
    progMode = FIXMODE;
    int temp = -1;
    #else
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
    #endif
    
    _pinMode( modePin, OUTPUT );
    
    #ifdef DEBUG
    #warning "Debugging ist aktiv"
    Serial.begin(115200); //Debugging
        #if defined(__STM32F1__) || defined(__AVR_ATmega32U4__) 
        // auf STM32/ATmega32u4: warten bis USB aktiv (maximal 6sec)
        {  unsigned long wait=millis()+6000;
           while ( !Serial && (millis()<wait) );
        }
        #endif
        #ifdef LOCONET
           DB_PRINT(  ">>>>>>>>>> Neustart: (SV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
        #else
           DB_PRINT(  ">>>>>>>>>> Neustart: (CV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
        #endif    

     Serial.print( "Betr:" ); Serial.print(temp);Serial.print(" -> Mode=" );
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
    #endif // Ende #ifdef DEBUG

    //-------------------------------------
    // CV's initiieren
    if ( (ifc_getCV( CV_MODEVAL )&0xf0) != VALIDFLG || ifc_getCV(CV_INIVAL) != VALIDFLG || analogRead(resModeP) < 100 ) {
        // In modeVal oder ManufactId steht kein korrekter Wert ( oder resModeP ist auf 0 ),
        // alles initiieren mit den Defaultwerten
        // Wird über DCC ein 'factory-Reset' empfangen wird modeVal zurückgesetzt, was beim nächsten
        // Start zum initiieren führt.
        //
        DB_PRINT( "Init-ALL!!",0);
        iniCv( INIALL );
    } else if ( progMode == INIMODE ) {
        iniCv( INIMODE );
    }
    //-----------------------------------
    // Interface initiieren
    ifc_init( DCC_DECODER_VERSION_ID, progMode, CV_POMLOW );
    
    // Betriebsart auslesen
    opMode = ifc_getCV( CV_MODEVAL) &0x0f;
    rocoOffs = ( opMode & ROCOADDR ) ? 4 : 0;
    DB_PRINT( "opMode=%d , rocoOffs=%d", opMode, rocoOffs );

    // Encoder-Init
    IniEncoder();

    //--- Ende Grundinitiierung ---------------------------------
    
    setWeichenAddr(); // 1. Weichenadresse berechnen
        
    #ifdef DEBUG
    byte *heap_start = new( byte );
    #endif
    //--- Die definierten Io's in einem linearen Array speichern. Der entsprechende Array-Abschnitt
    // wird den Funktionsobjekten als Pointer übergeben, die damit ein Array mit 'ihren' Pin-Nummern
    // erhalten
    for ( byte wIx = 0; wIx<WeichenZahl; wIx++ ){
        ioPins[wIx*PPWA] = out1Pins[ wIx ];
        ioPins[wIx*PPWA+1] = out2Pins[ wIx ];
        ioPins[wIx*PPWA+2] = out3Pins[ wIx ];
    }
    //--- Funktionsobjekte entsprechend der Konfiguration instanziieren ------------------
    for ( byte wIx=0; wIx<WeichenZahl; wIx++ ) {
        // Funktionsobjekte instanziieren und initiieren
        byte vsIx = 0;  // Vorsignalindex am Mast auf 0 (kein Vorsignal) vorbesetzen
        adressTyp[wIx] = NOCOM;  // Standard ist keine Folgeadresse
        switch (iniTyp[wIx] )  {
          case FSERVO:
            // 1. Servo immer einrichten
            Fptr.servo[wIx] = new Fservo( cvAdr(wIx,0) , &ioPins[wIx*PPWA] );
            // Prüfen ob Servokombination ( Folgetyp = FSERVO0 )
            if ( wIx+1<WeichenZahl && iniTyp[wIx+1] == FSERVO0 ){
                // prüfen ob FSERVO0 gleicher Anschlußpin ( =Servo mit mehreren Stellungen )
                if ( out1Pins[wIx] != out1Pins[wIx+1] ) {
                    // nein, 2 Servos mit kombinatorischer Ansteuerung
                    // das 2. Servo greift auf das Modbyte des 1. Servos zu, da das Mod-Byte
                    // des 2. Servos die Stellungskombinatorik enthält
                    Fptr.servo[wIx+1] = new Fservo( cvAdr(wIx+1,0) , &ioPins[(wIx+1)*PPWA], -CV_BLKLEN );
                    adressTyp[wIx] = SERVO_DOUBLE;
                } else {
                    adressTyp[wIx] = SERVO4POS;
                }
            }
            break;
          case FCOIL:
            Fptr.coil[wIx] = new Fcoil( cvAdr(wIx,0) , &ioPins[wIx*PPWA] );
            break;
          case FSTATIC:
            Fptr.stat[wIx] = new Fstatic( cvAdr(wIx,0) , &ioPins[wIx*PPWA] );
            break;
          case FSIGNAL2:
            // Prüfen ob ein Vorsignal am Mast Dunkelgeschaltet werden muss
            vsIx = getCvPar(wIx,PAR3);
            if( !(vsIx >= 1 && vsIx <= WeichenZahl) ) {
                // kein gültiger Vorsignalindex gefunden
                vsIx = 0;
            }
            [[fallthrough]];
            // die restliche Bearbeitung ist bei Signalen und Vorsignalen gleich
          case FVORSIG:
            {   // Zahl der Ausgangspins (PPWA*Folgeadressen) bestimmen
                byte pinZahl = PPWA; 
                if ( wIx+1<WeichenZahl && iniTyp[wIx+1] == FSIGNAL0 ){
                    pinZahl+=PPWA;
                    if ( wIx+2<WeichenZahl && iniTyp[wIx+2] == FSIGNAL0 ) {
                        pinZahl+=PPWA;
                        adressTyp[wIx] = SIGNAL3ADR;    // Lichtsignal mit 3 Adressen
                    } else {
                        adressTyp[wIx] = SIGNAL2ADR;    // Lichtsignal mit 2 Adressen                        
                    }
                }
                DBSG_PRINT("Signal %d, PinMax=%d, Vsindex: %d",wIx+1, pinZahl, vsIx );
                if ( vsIx == 0 ) {
                    Fptr.sig[wIx] = new Fsignal( cvAdr(wIx,0) , &ioPins[wIx*PPWA], pinZahl, NULL );
                } else {
                    Fptr.sig[wIx] = new Fsignal( cvAdr(wIx,0) , &ioPins[wIx*PPWA], pinZahl, &Fptr.sig[vsIx-1] );
                }
            }
            break;
          default: // auch FSIGNAL0, FSERVO0
            // hier werden gegebenenfalls Servo- und Signalfolgetypen übersprungen
            ;
        } // Ende switch Funktionstypen
    } // Ende loop über alle Funktionen

    DBprintCV(); // im Debug-Mode alle CV-Werte ausgeben
#ifdef DEBUG
    byte *heap_end = new( byte );
#ifdef __AVR_MEGA__
    // Für Test Speicherbelegung ausgeben ( beim Heap sind die Adressen auf das RAM bezogen
    // der Offset von 256 Byte IO-Adressen im ATMEga 328 wird abgezogen
    DB_PRINT(">> Setup-Ende >> Heap: Start=0x%x (%d), End=0x%x (%d)", (int)&__heap_start-256, (int)__malloc_heap_start-256, (int)__brkval-256,(int)__brkval-256);
#endif
    DB_PRINT(">>HEAP>> Start=0x%x, End=0x%x, Size=%d", heap_start, heap_end, heap_end-heap_start );

#endif
}



////////////////////////////////////////////////////////////////
void loop() {
    #ifdef DEBUG
    /*static unsigned long startMicros = micros();;
    static int loopCnt = 0;
    loopCnt++;
    if ( micros() - startMicros > 1000000L ) {
        // jede Sekunde Loopdauer ausgeben
        Serial.print( "Loopdauer(µs):"); Serial.println( 1000000L/(long)loopCnt );
        startMicros = micros();
        loopCnt = 0;
    }*/
//    dccSim();       // Simulation von DCC-Telegrammen
    #endif
    
    getEncoder();   // Drehencoder auswerten und Servolage gegebenenfalls anpassen
    ifc_process();  // Hier werden die empfangenen Telegramme analysiert und der Sollwert gesetzt
    #ifdef DEBUG
    // Merker CV für CV-Ausgabe rücksetzen (MerkerCV ist 1.CV hinter dem CV-Block für die ausgangskonfiguration)
    if( ifc_getCV( cvAdr(WeichenZahl,MODE)) != 0xff ) ifc_setCV( cvAdr(WeichenZahl,MODE) , 0xff );
    #endif
    
    // Ausgänge ansteuern
    for ( byte i=0; i<WeichenZahl; i++ ) {
        switch ( iniTyp[i]  ) {
          case FSERVO: // Servoausgänge ansteuern ----------------------------------------------
            Fptr.servo[i]->process();
            if ( adressTyp[i] == SERVO_DOUBLE ) {
                //FolgeServo ansteuern
                Fptr.servo[i+1]->process();
            }
            break;
          case FCOIL: //Doppelspulenantriebe ------------------------------------------------------
           //if ( dccSoll[i] != SOLL_INVALID ) DBCL_PRINT( "SollCoil=%d", dccSoll[i] );
           Fptr.coil[i]->process();
            break;
          case FSTATIC: // Ausgang statisch ein/ausschalten ------------------------------------
            Fptr.stat[i]->process();
            break;
          case FVORSIG:
          case FSIGNAL2:
            Fptr.sig[i]->process();
            break;
          case FSIGNAL0:
          case FSERVO0: 
            // Signalfolgetypen überspringen
            ;
        } // - Ende Switch Funktionstypen-------------------------------------------
    } // Ende Schleife über die Funktionen (Weichen)---------------


    #ifndef LOCONET
        #ifndef NOACK
        // nur DCC: Ackimpuls abschalten--------------------------
        if ( !AckImpuls.running() ) _digitalWrite( ackPin, LOW );
        #endif
    #else
    // nur bei Loconet: prüfen ob Loconet-Id geändert werden soll
    if ( chgLoconetId && !idLoconet.running() ) {
        chgLoconetId = false;
        ifc_init( CV_POMLOW );    }
    #endif
    
    // Programmierled blinkt im Programmiermode bis zum Empfang einer Adresse
    if ( ! ledTimer.running() && progMode == ADDRMODE) {
        ledTimer.setTime( 500 );
        if ( digitalRead( modePin ) ) 
            CLR_PROGLED;
        else
            SET_PROGLED;
    }
} // Ende loop


//-------------- Setzen der Funktionssollwerte ------------------------
// Dieses Unterprogramm wird aufgerufen, wenn ein Funktionsobjekt umgeschaltet
// werden soll. z.B. Signalbild am Lichtsignal, oder Weiche umschalten.
void setPosition( byte wIx, byte sollWert, byte state = 0 ) {
    // bei den meisten Funktionen ist für den Sollwert nur 0/1 sinnvoll. Bei Lichtsignalen
    // mit mehreren Signalbildern sind auch höhere Werte sinnvoll ( bis zu 0...5 bei 6
    // Signalbildern)
    // state wird nur von FCOIL ausgewertet
    DB_PRINT("Set wIx=%d, soll=%d, state=%d", wIx, sollWert, state);
    switch ( iniTyp[wIx] ) {
        case FSERVO:
          // prüfen, ob es zwei in Kombination anzusteuernde Servos sind
          if ( adressTyp[wIx] == SERVO_DOUBLE ) {
            // ja, Positionen aus dem Modbyte des 2. Servos bestimmen.
            byte pos = ifc_getCV( CV_FUNCTION + CV_BLKLEN*(wIx+1) ) >> ( sollWert*2 );
            DBSV_PRINT("Stellbyte=%02X",pos);
            Fptr.servo[wIx]->set( pos&1 );
            pos >>= 1;
            Fptr.servo[wIx+1]->set( pos&1 );
          } else {
            // Standardservo oder Mehrstellungsservo
            Fptr.servo[wIx]->set( sollWert );
          }
          break;
        case FSTATIC:
          Fptr.stat[wIx]->set( sollWert );
          break;
        case FCOIL:
          Fptr.coil[wIx]->set( sollWert, state );
          break;
        case FSIGNAL2:
        case FVORSIG:
          Fptr.sig[wIx]->set( sollWert );
          break;
    }
}
//////////////////////////////////////////////////////////////
// Unterprogramme, die von der DCC bzw. LocoNet Library aufgerufen werden:
//------------------------------------------------
// Die folgende Funktion wird von ifc_process() aufgerufen, wenn ein Weichentelegramm empfangen wurde
void ifc_notifyDccAccState( uint16_t Addr, uint8_t OutputAddr, uint8_t State ){
    // Weichenadresse berechnen
    byte i,dccSoll,dccState;
    uint16_t wAddr = Addr+rocoOffs; // Roco zählt ab 0, alle anderen lassen die ersten 4 Weichenadressen frei
    // Im Programmiermodus bestimmt das erste empfangene Telegramm die erste Weichenadresse
    if ( progMode == ADDRMODE ) {
        // Adresse berechnen und speichern
        // Decoder ist immer im Output-Adressmode
        weichenAddr = wAddr;
        ifc_setCV( cvAccDecAddressLow, wAddr%256 );
        ifc_setCV( cvAccDecAddressHigh, wAddr/256 );
        progMode = PROGMODE;
        SET_PROGLED;
       DB_PRINT( "Neu: 1.Weichenaddr: %d ", weichenAddr );
    }
    // Testen ob eigene Weichenadresse
    DB_PRINT( "Weichenadresse: %d , Ausgang: %d, State: %d", wAddr, OutputAddr, State );
    // Prüfen ob Adresse im Decoderbereich
    if ( wAddr >= weichenAddr && wAddr < (weichenAddr + WeichenZahl) ) {
        // ist eigene Adresse, Sollwert setzen
        byte Ix = wAddr-weichenAddr;
        dccSoll =  OutputAddr & 0x1;
        dccState = State;
        
       
        //DB_PRINT( "Weiche %d, Index %d, Soll %d, Ist %d", wAddr, Ix, dccSoll[Ix],  fktStatus[Ix] );
        // Bei Servo- und Signaladressen muss der Sollzustand gegebenenfalls angepasst werden: Bei der ersten
        // Folgeadresse von 0/1 auf 2/3, bei der 2. Folgeadresse auf 4/5 ( Signale können bis zu 6
        // Sollzustände haben ). Ausserdem muss dieser geänderte Sollzustand an die Grundadresse
        // übergeben werden.
        if ( iniTyp[Ix] == FSIGNAL0 || iniTyp[Ix] == FSERVO0 ) {
            // es ist eine Folgeadresse
            dccSoll += 2;
            Ix--;           // Index auf Grundadresse stellen
            if ( iniTyp[Ix] == FSIGNAL0 ) {
                // ist 2. Folgeadresse
                 dccSoll += 2;
                 Ix--;
            }
        }
        // Prüfen ob gerade ein Servo justiert wird, und gebenenfalls die Justierung beenden 
        ChkAdjEncode( Ix, dccSoll );

        // angesteuerte Adresse einstellen
        setPosition( Ix, dccSoll, dccState );
    }
    // Prüfen ob Vorsignal über Hauptsignaladresse geschaltet werden muss
    for ( i = 0; i < WeichenZahl; i++ ) {
        uint16_t vsAdr;
        if ( iniTyp[i] == FVORSIG  ) {
            // Adresse des zugehörigen Hauptsignals bestimmen
            vsAdr = getCvPar(i, PAR3) + 256 * getCvPar(i, STATE);
            if ( vsAdr == wAddr ) {
                DBSG_PRINT( "Vorsig0 %d, Index %d, Soll %d", wAddr, i, OutputAddr & 0x1 );
                setPosition( i, OutputAddr & 0x1 );
                break; // Schleifendurchlauf abbrechen, es kann nur eine Signaladresse sein
            } else {
                // Folgeadresse ( bei mehrbegriffigen Vorsignalen ) prüfen
                if ( i+1 < WeichenZahl && iniTyp[i+1] == FSIGNAL0 ) {
                    // Folgeadresse vergleichen
                    if ( vsAdr+1 == wAddr ) { 
                        // Übereinstimmung gefunden, neues Signalbild setzen
                        DBSG_PRINT( "Vorsig1 %d, Index %d, Soll %d", wAddr, i, (OutputAddr & 0x1)+2  );
                        setPosition( i, (OutputAddr & 0x1)+2 );
                    }
                }
            }
        }
    }
}
//---------------------------------------------------
#ifndef LOCONET
// wird aufgerufen, wenn die Zentrale ein CV ausliest. Es wird ein 60mA Stromimpuls erzeugt
void ifc_notifyCVAck ( void ) {
    #ifndef NOACK
    // Ack-Impuls starten
    //DB_PRINT( "Ack-Pulse" );
    AckImpuls.setTime( 6 );
    _digitalWrite( ackPin, HIGH );
    #endif
}
#endif
//-----------------------------------------------------
// Wird aufgerufen, nachdem ein CV-Wert verändert wurde
void ifc_notifyCVChange( uint16_t CvAddr, uint8_t Value ) {
    if ( !localCV ) {
        //CV wurde über nmraDCC geändert. Ist dies eine aktive Servoposition, dann die Servoposition
        // entsprechend anpassen
       DB_PRINT( "neu: CV%d=%d", CvAddr, Value );
        for ( byte i=0; i<WeichenZahl; i++ ) {
            // prüfen ob Ausgang einen Servo ansteuert:
            switch ( iniTyp[i] ) {
              case FSERVO:
                // gehört der veränderte CV zu diesem Servo?
                if (  (CvAddr == cvAdr(i,PAR1) && Fptr.servo[i]->getPos() == GERADE) ||
                      (CvAddr == cvAdr(i,PAR2) && Fptr.servo[i]->getPos() == ABZW ) ){
                    // Es handelt sich um die aktuelle Position des Servos,
                    // Servo neu positionieren
                    //DBSV_PRINT( "Ausg.%d , Pos. %d neu einstellen", i, dccSoll[i] );
                     Fptr.servo[i]->adjust( ADJPOS, Value );
                } else if ( CvAddr == cvAdr(i,PAR1) ||
                            CvAddr == cvAdr(i,PAR2)  ) {
                      // ist nicht de aktuelle Position des Servos, Servo umstellen
                      Fptr.servo[i]->set( ! Fptr.servo[i]->getPos() );
                } else if ( CvAddr == cvAdr(i,PAR3) ) {
                    // die Geschwindigkeit des Servo wurde verändert
                    //DBSV_PRINT( "Ausg.%d , Speed. %d neu einstellen", i, Value );
                    Fptr.servo[i]->adjust( ADJSPEED, Value );
                }
            }
        }
        #ifdef DEBUG
            // prüfen ob die CV-Adresse HINTER den Weichenadressen verändert wurde. Wenn ja,
            // alle CV-Werte ausgeben und Wert wieder auf 0xff setzen
            if ( CvAddr ==  cvAdr(WeichenZahl,MODE) && Value !=0xff ) {
                DBprintCV();
            }
        #endif
        if ( CvAddr == cv29Config ) {
          // CV29 darf nicht verändert werden -> auf default setzen
          ifc_setCV( cv29Config, config29Value ); // == Accessory-Decoder mit Output-Adressing
        }
        // prüfen ob die Weichenadresse verändert wurde. 
        // Dies kann durch Ändern der Decoderadresse geschehen.
        // Wird die Decoderadresse geändert muss zuerst das MSB (CV9) verändert werden. Mit dem
        // Ändern des LSB (CV1) wird dann die Weichenadresse neu berechnet
        //if (CvAddr ==  cvAccDecAddressLow || CvAddr ==  cvAccDecAddressHigh) setWeichenAddr();
        if (CvAddr ==  cvAccDecAddressLow ) setWeichenAddr();

        #ifdef LOCONET
        // Prüfen ob Pom-Adresse geändert wurde. Wenn ja, reset-Timer starten
        if ( CvAddr == CV_POMLOW || CvAddr == CV_POMHIGH ) {
            chgLoconetId = true;
            idLoconet.setTime( 2000 );
        }
        #endif
    }
}    
//-----------------------------------------------------
void ifc_notifyCVResetFactoryDefault(void) {
    // Auf Standardwerte zurücksetzen und Neustart
    ifc_setCV( CV_MODEVAL, 255 );
    delay( 20 );
    DB_PRINT( "Reset: modeVal=0x%2x", ifc_getCV( CV_MODEVAL ) );
    delay(500);
    softReset();
}
//------------------------------------------------------
void ifc_notifyDccReset( uint8_t hardReset ) {
#ifdef DEBUG
    //if ( hardReset > 0 )//DB_PRINT("Reset empfangen, Value: %d", hardReset);
    // wird bei CV-Auslesen gesendet
#endif
}
//--------------------------------------------------------

/////////////////////////////////////////////////////////////////////////
// Initiieren der CV-Werte
void iniCv( byte mode ) {
        localCV= true; // keine auswertung in ifc_notifyCVchange
        // Standard-CV's
        DB_PRINT("iniCV: %d", mode );
        for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
                ifc_setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
        }
        // Decoderspezifische CV's

        // allgemeine CV's
        ifc_setCV( (int) CV_POMLOW, PomAddr%256 );
        ifc_setCV( (int) CV_POMHIGH, PomAddr/256 );
        ifc_setCV( (int) CV_INIVAL, VALIDFLG );
        ifc_setCV( (int) CV_MODEVAL, VALIDFLG | (iniMode&0xf) );
        // Funktionsspezifische CV's
        for ( byte i = 0; i<WeichenZahl; i++ ) {
            DB_PRINT("fktSpezCv: %d,Typ=%d", i, iniTyp[i] );
            ifc_setCV( cvAdr(i,MODE), iniFmode[i] );
            ifc_setCV( cvAdr(i,PAR1), iniPar1[i] );
            ifc_setCV( cvAdr(i,PAR2), iniPar2[i] );
            ifc_setCV( cvAdr(i,PAR3), iniPar3[i] );
            if ( mode == INIALL ) {
                // Bei INIALL auch alle Statuswerte initiieren
                ifc_setCV( cvAdr(i,STATE), iniPar4[i] );
            }  else {
                // bei den Signaltypen immer auch den 5. CV-Wert als Parameter laden
                switch ( iniTyp[i] ) {
                  case FSIGNAL2:
                  case FVORSIG:
                  case FSIGNAL0:
                    ifc_setCV( cvAdr(i,STATE), iniPar4[i] );
                    break;
                  default:
                    ;
            }
        }
    }
    localCV=false;
}
//-------------------------------------------------------
// Unterprogramme zur Servojustierung mit Drehencoder
void IniEncoder( void ) {
    #ifdef ENCODER_AKTIV
    // Encoder initiieren
    _pinMode( encode1P, INPUT_PULLUP );
    _pinMode( encode2P, INPUT_PULLUP );
    AdjServo = NULL;
    adjPulse = NO_ADJ;
    #endif
}
   
void getEncoder(  ) {
    #ifdef ENCODER_AKTIV
    
    static bool dirState, taktState;
    static int lastCnt,jogCount = 0;
    static enum {IDLE0, TAKTHIGH, TAKTLOW } encoderState;
    // Encoder-Statemachine
    switch ( encoderState ) {
      case IDLE0: // Grundstellung, warten auf Statuswechsel an encode1P
        if ( digitalRead( encode1P )!= dirState ) {
            bool temp = digitalRead( encode2P );
            if (  temp != taktState ) {
                // Es gab eine zusätzliche Taktflanke, Richtungsumkehr!
                //Serial.println( "++++Doppeltakt" );
                jogCount = lastCnt;
            }
            // Taktinput wieder scharf
            encoderState = digitalRead( encode2P )? TAKTHIGH : TAKTLOW;
        }
        break;
      case TAKTHIGH:
        // warte auf neg. Flanke am Taktinput
        if ( ! digitalRead( encode2P )  ) {
            // Bei Schaltern mit doppelter Rastung auch hier Puls erzeugen
            taktState = LOW;
            dirState = digitalRead( encode1P );
            lastCnt = jogCount;
            # ifdef ENCODER_DOUBLE
            if ( dirState )
                jogCount++;
            else jogCount--;
            #endif
            encoderState = IDLE0;
        } 
        break;
      case TAKTLOW:
        // warte auf pos. Flanke am Taktinput
        if ( digitalRead( encode2P )  ) {
            taktState = HIGH;
            // Richtung bestimmen
            dirState = digitalRead( encode1P );
            lastCnt = jogCount;
            if ( dirState )
                jogCount--;
            else jogCount++;
            encoderState = IDLE0;
        } 
    }
    #ifdef DEBUG
    // encoderzähler ausgeben
    if ( jogCount != 0 ) {
        DBSV_PRINT( "Encoder: %d", jogCount );        
    }
    #endif

    if ( AdjServo != NULL && !AdjServo->isMoving() ) {
        // es gibt eine aktuell zu justierendes Servo, das sich nicht
        // gerade bewegt
        if ( jogCount != 0 ) {
            // Drehencoder wurde bewegt 
            if ( adjPulse == NO_ADJ ) {
                // ist erster Justierimpuls, Servo auf aktuelle Justierposition stellen
                // und aktuelle Position aus CV auslesen
                AdjServo->set( adjPos );
                adjPulse = AdjServo->getCvPos();
            }
            if ( (jogCount>0 && adjPulse<180) || (jogCount<0 && adjPulse>0) )
                adjPulse += jogCount; // adjPulse nur im Bereich 0...180 
            AdjServo->adjust( ADJPOS, adjPulse );
        } else if ( analogRead( resModeP ) < 500 ) {
            // Mittelstellungstaster gedrückt
            DBSV_PRINT("Servo center",0);
            AdjServo->center(ABSOLUT);
        }
    }
    jogCount = 0;
    #endif
}

void ChkAdjEncode( byte WIndex, byte dccSoll ){
    #ifdef ENCODER_AKTIV
    // Prüfen, ob die Folgeadresse zweier verbundener Servos angesprochen wurde
    if ( adressTyp[WIndex] == SERVO_DOUBLE && dccSoll > 1 ) {
        // Folgeadresse wurde angesprochen, Index auf dieses Servo stellen
        WIndex++;
        dccSoll -= 2;
    }
    // nach dem Empfang einer Weichenadresse wird geprüft, ob eine vorherige Justierung gespeichert werden
    // muss. Die empfangene Weichenadresse wird als neue Justieradresse gespeichert wenn es sich um einen
    // Servoantrieb handelt.
    if ( adjPulse != NO_ADJ ) {
        // Es wurde justiert, testen ob gespeichert werden muss (Servowechsel)
        if ( Fptr.servo[WIndex] != AdjServo || adjPos != dccSoll ) {
            // Weiche wurde umgeschaltet, oder eine andere Weiche betätigt -> Justierung speichern
            localCV = true; // keine Bearbeitung in Callback-Routine NotifyCvChange
            AdjServo->adjust(ADJPOSEND,adjPulse);
            adjPulse = NO_ADJ;
            AdjServo = NULL;
            localCV = false;
        }
    }
    if ( iniTyp[ WIndex ] == FSERVO || iniTyp[ WIndex] == FSERVO0 ) {
        // die neue Adresse bezieht sich auf einen Servo
        AdjServo = Fptr.servo[WIndex];
        adjPos = dccSoll;
    } else {
        AdjServo = NULL;
    }
    #endif
    DB_PRINT("chkAdj-Freemem %d", freeMemory() );

}




//////////////////////////////////////////////////////////////////////////
// Allgemeine Unterprogramme 
void setWeichenAddr(void) {
    // Decoder ist immer im Ouput-Adressmode
        weichenAddr = ifc_getAddr( );
   /* else
        weichenAddr = (ifc_getAddr( )-1)*4 +1 + rocoOffs ;*/
    DB_PRINT("setWadr: getAdr=%d, wAdr=%d", ifc_getAddr(), weichenAddr );
}
//--------------------------------------------------------
void softReset(void){
    DB_PRINT( "RESET", 0 );
    delay(1000);
    #ifdef __AVR_MEGA__
    //asm volatile ("  jmp 0");
    cli(); //irq's off
    wdt_enable(WDTO_15MS); //wd on,15ms
    while(1); //loop              break;
    #endif
    #ifdef __STM32F1__
    nvic_sys_reset();   //System-Reset der STM32-CPU
    #endif
}
//-----------------------------------------------------------
#ifdef DEBUG
/* Simulation der DCC-Befehle per serieller Schnittstelle

"ac 14 1 0"  > Zubehöradresse 14, sollwert 1 stat 0
"cr 50"      > Cv-Adresse 50 lesen
"cw 50 3"    > Cv Adresse 50 mit Wert 3 beschreiben

Es werden die gleichen Callbacks wie von der nmraDCC Lib aufgerufen
*/
char rcvBuf[20];
byte rcvIx=0;       // Index im Empfangspuffer
void dccSim ( void ) {
    // Wenn Daten verfügbar, diese in den receive-Buffer lesen. Endezeichen ist LF oder CR
    char *token;
    int adr =0; // empfagnee Adresse
    byte soll,state;
    byte dataAnz = Serial.available();
    if ( dataAnz > 0 ) {
        Serial.readBytes( &rcvBuf[rcvIx], dataAnz );
        rcvIx += dataAnz;
        if ( rcvBuf[rcvIx-1] == 10 || rcvBuf[rcvIx-1] == 13 ) {
            rcvBuf[rcvIx] = 0;
            // komplette Zeile empfangen -> auswerten
            token = strtok( rcvBuf, " ,");
            if ( strcmp( token, "ac" ) == 0 ) {
                // Zubehörbefehl
                adr = atoi( strtok( NULL, " ," ) );
                soll = atoi( strtok( NULL, " ," ) );
                state = atoi( strtok(NULL, " ,") );
                DB_PRINT("Sim: AC,%d,%d,%d",adr,soll,state);
                ifc_notifyDccAccState( adr, soll, state );           }
        // Empfangspuffer rücksetzen
        rcvIx = 0;
        }
    }
}

#ifdef __AVR_MEGA__
int freeMemory() {
    // die Adressen des RAM beginnen bei 256 (0x100). Darunter liegen die
    // IO-Adressen.
    int free_memory;
    //DB_PRINT( "&fremem=%d, &heapstrt=%d, heapStrt=%d, brkVal=%d",&free_memory, &__heap_start, __heap_start, __brkval);
    if ((int)__brkval == 0) {
        free_memory = ((int)&free_memory) - ((int)&__heap_start);
    } else {
        free_memory = ((int)&free_memory) - ((int)__brkval);
    }
    return free_memory;
}
#else
int freeMemory() {
    return 0;
}
#endif

void DBprintCV(void) {
    // für Debug-Zwecke den gesamten genutzten CV-Speicher ausgeben
    // Standard-Adressen
   DB_PRINT( "--------- Debug-Ausgabe CV-Werte ---------", 0 );
   DB_PRINT( "Version: %x, ManufactId: %d", ifc_getCV( cvVersionId ), ifc_getCV( cvManufactId ) );
    
    // Decoder-Konfiguration global
   #ifdef LOCONET
   DB_PRINT(  "Initval (SV45/47) : 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
    DB_PRINT( "Konfig   (SV29)   : 0x%X", ifc_getCV( cv29Config ) );
    DB_PRINT( "Adresse:(SV17/18) : %d", ifc_getCV( cvAccDecAddressLow )+ifc_getCV( cvAccDecAddressHigh )*256);
    DB_PRINT( "LoconetId(SV48/49): %d"   , ifc_getCV( CV_POMLOW) + 256* ifc_getCV( CV_POMHIGH ) );
   #else
   DB_PRINT(  "Initwert (CV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
    DB_PRINT( "Konfig   (CV29)   : 0x%X", ifc_getCV( cv29Config ) );
    DB_PRINT( "Adresse:(CV1/9)   : %d", ifc_getCV( cvAccDecAddressLow )+ifc_getCV( cvAccDecAddressHigh )*256);
    DB_PRINT( "PoM-Adr.(CV48/49) : %d"   , ifc_getCV( CV_POMLOW) + 256* ifc_getCV( CV_POMHIGH ) );
   #endif    
    // Output-Konfiguration
   DB_PRINT( "Wadr | Typ | CV's  | Mode | Par1 | Par2 | Par3 | Status |",0 );
    for( byte i=0; i<WeichenZahl; i++ ) {
       DB_PRINT( "%4d |%2d/%1d | %2d-%2d | %4d | %4d | %4d | %4d | %3d " , weichenAddr+i, iniTyp[i],adressTyp[i],
                                                                 cvAdr(i,MODE),  cvAdr(i,STATE),
                                                                 getCvPar(i,MODE),
                                                                 getCvPar(i,PAR1),
                                                                 getCvPar(i,PAR2),
                                                                 getCvPar(i,PAR3),
                                                                 getCvPar(i,STATE) );
    }
    
}
#else
void DBprintCV(void) {
    
}
#endif

/* DIY Zubehördecoder
 *
 * Klassen für die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * Für jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 */
 // #define DEBUG ;
#include "FuncClasses.h"

extern NmraDcc Dcc;


//---------------allgemeine Hilfsfunktionen -------------------------------
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode ) {
#else
void _pinMode( byte port, byte mode ) {
#endif
    if ( port != NC ) pinMode( port,  mode );
}

void _digitalWrite( byte port, byte state ) {
    if( port != NC ) digitalWrite( port, state );
}


//----------------------- FCOIL ---------------------------------------------

Fcoil::Fcoil( int cvAdr, uint8_t out1P[] ) {
    // Konstruktor für Doppelspulenantriebe
    _cvAdr = cvAdr;
    _outP = out1P;
    DB_PRINT( "Fcoil CV=%d, OutPins %d,%d ", _cvAdr, _outP[0], _outP[1] );
    
    for ( byte i=0; i<2; i++ ) {
        pinMode( _outP[i], OUTPUT );
        digitalWrite( _outP[i], LOW );
    }
    _pulseON = false;
}
    
    
void Fcoil::chkState( uint8_t *dccSoll, uint8_t dccState ) {
        // Pulseausgägne einschalten
    if (  !_pulseON && !_pulseT.running() &&  *dccSoll != SOLL_INVALID ) {
        // Aktionen am Ausgang nur wenn kein aktiver Impuls und der Pausentimer nicht läuft
        DB_PRINT(" Ist=%d, Soll=%d ", _fktStatus, *dccSoll );
        if ( ( _fktStatus != *dccSoll || (Dcc.getCV(_cvAdr+MODE) & NOPOSCHK) )
                && dccState ) {
            // Weiche soll geschaltet werden
            DB_PRINT(" State=%d",  dccState );
            if ( (*dccSoll & 1) == 0 ) {
                // Out1 aktiv setzen
                digitalWrite( _outP[0], HIGH );
                digitalWrite( _outP[1], LOW );
                //DB_PRINT( "Pin%d HIGH, Pin%d LOW", _outP[0], _outP[1] );
            } else {
                // Out2 aktiv setzen
                digitalWrite( _outP[1], HIGH );
                digitalWrite( _outP[0], LOW );
                //DB_PRINT( "Pin%d LOW, Pin%d HIGH", _outP[0], _outP[1] );
            }
            _pulseON = true;
            if ( Dcc.getCV(_cvAdr+PAR1) > 0 ) _pulseT.setTime( Dcc.getCV(_cvAdr+PAR1) * 10 );
            _fktStatus = *dccSoll;
            Dcc.setCV( _cvAdr+STATE, *dccSoll );
        }
        *dccSoll = SOLL_INVALID; // Empfangenes Telegramm wurde bearbeitet.
    }
    
    // Pulsausgänge ausschalten
    if ( _pulseON ) {
        // prüfen ab Impuls abgeschaltet werden muss
        if ( !(Dcc.getCV(_cvAdr+MODE) & CAUTOOFF) && *dccSoll!=SOLL_INVALID && ! dccState ) {
            // ein Abschalttelegramm wurde empfangen
            _pulseON = false;
            _pulseT.setTime( 0 );     // Timer abschalten, falls er läuft
            *dccSoll = SOLL_INVALID;  // Empfangenes Telegramm wurde bearbeitet.
        }
        // Timerabschaltung
        if ( (Dcc.getCV(_cvAdr+PAR1) > 0) && ! _pulseT.running()  ) {
            //  Timerabschaltung ist aktiv und Timer ist abgelaufen
            //DB_PRINT( "Pin%d LOW, Pin%d LOW", _outP[0], _outP[1] );
            _pulseON = false;
        }
        
        if ( _pulseON == false ) {
            digitalWrite( _outP[0], LOW );
            digitalWrite( _outP[1], LOW );
            // Timer für Pulspause setzen
            if ( Dcc.getCV(_cvAdr+PAR2) > 0 ) _pulseT.setTime( Dcc.getCV(_cvAdr+PAR2) * 10 );
        }
        
    }

}


//------------------------FSTATIC -------------------------------------------- 

Fstatic::Fstatic( int cvAdr, uint8_t ledP[] ) {
    // Konstruktor der Klasse für statisches Leuchten bzw. blinken
    _cvAdr = cvAdr;
    _ledP = ledP;
    DB_PRINT( "Fstatic CV=%d, LedPis %d,%d ", _cvAdr, _ledP[0], _ledP[1] );
    _fktStatus = Dcc.getCV( _cvAdr+STATE );
    // Modi der Ausgangsports
    if ( Dcc.getCV( _cvAdr+MODE ) & BLKSOFT ) {
        // Ausgangsports als Softleds einrichten
        for ( byte i=0; i<2; i++ ) {
            if ( _ledP[i] != NC ) {
                _ledS[i] = new SoftLed;
                byte att, rise, writ;
                att=_ledS[i]->attach( _ledP[i] );
                _ledS[i]->riseTime( 500 );
                _ledS[i]->write( OFF, LINEAR );
               DB_PRINT( "Softled, pin %d, Att=%d", _ledP[i], att );
            }
        }
    } else {
        if ( _ledP[0] != NC ) pinMode( _ledP[0], OUTPUT );
        if ( _ledP[1] != NC ) pinMode( _ledP[1], OUTPUT );
        DB_PRINT( "Hardled, pins %d,%d ", _ledP[0], _ledP[1] );
    }
    // Grundstellung der Ausgangsports
    if ( Dcc.getCV( _cvAdr+MODE ) & BLKMODE ) {
        // aktuellen Blinkstatus berücksichtigen
        _setLedPin(0, LOW );
        _setLedPin(1, LOW );
    } else {
        // statische Ausgabe
        _setLedPin(0, _fktStatus & 0x1 );
        _setLedPin(1, !(_fktStatus & 0x1  ) );
    }
}

void Fstatic::_setLedPin( uint8_t ledI, uint8_t sollWert ) {
    // den LED ausgang 1/2 setzen. je nach Konfiguration als Softled oder hart
    if ( ledI <2 ) {
        if ( _ledS[ledI] != NULL ) _ledS[ledI]->write( sollWert);
        else if ( _ledP[ledI] != NC ) digitalWrite( _ledP[ledI], sollWert );
    }
}
void Fstatic::chkState( uint8_t *sollWert ) {
    // muss Ausgang umgeschaltet werden?
    if ( *sollWert!=SOLL_INVALID && (*sollWert&1) != (_fktStatus&1) ) {
        _setLedPin(0, *sollWert );
        if ( Dcc.getCV( _cvAdr+MODE) & BLKMODE ) {
            _setLedPin(1, (Dcc.getCV( _cvAdr+MODE ) & BLKSTRT)&& (*sollWert&1) ); 
        } else {                   
            _setLedPin(1, !*sollWert );                    
        }
        DB_PRINT( "Fkt=%d, Soll=%d, Ist=%d", _cvAdr, *sollWert, _fktStatus );
        _fktStatus = *sollWert;
        Dcc.setCV( _cvAdr+STATE, _fktStatus );
        if ( _fktStatus && ( Dcc.getCV( _cvAdr+MODE ) & BLKMODE ) ) {
            // Funktion wird eingeschaltet und Blinkmode ist aktiv -> Timer setzen
            pulseT.setTime( Dcc.getCV( _cvAdr+PAR3)*10 );
            DB_PRINT( "BlkEin %d/%d, Strt=%x", Dcc.getCV( _cvAdr+PAR1) , Dcc.getCV( _cvAdr+PAR2), (Dcc.getCV( _cvAdr+MODE) & BLKSTRT)  );
            _fktStatus |= BLKON;
        }
        *sollWert = SOLL_INVALID;
    }
    if ( _fktStatus && ( Dcc.getCV( _cvAdr+MODE ) & BLKMODE ) ) {
        // bei aktivem Blinken die Timer abfragen/setzen
        if ( !pulseT.running() ) {
            // Timer abgelaufen, Led-Status wechseln
            if ( _fktStatus & BLKON ) {
                // Led ausschalten
                _setLedPin(0, LOW );
                _setLedPin(1, HIGH );
                _fktStatus &= ~BLKON;
                pulseT.setTime( Dcc.getCV( _cvAdr+PAR2 )*10 );
            } else {
                // Led einschalten
                _setLedPin(0, HIGH );
                _setLedPin(1, LOW );
                _fktStatus |= BLKON;
                pulseT.setTime( Dcc.getCV( _cvAdr+PAR1 )*10 );
            }
        }
    }
}

//----------------------------- FSERVO --------------------------------------------
Fservo::Fservo( int cvAdr, uint8_t pins[] ) {
    // Konstruktor der ServoKlasse
    _outP = pins;
    _cvAdr = cvAdr;
    // Weichenservo einrichten
    if ( _outP[SERVOP] != NC ) {
        _weicheS.attach( _outP[SERVOP], Dcc.getCV( _cvAdr+MODE ) & SAUTOOFF );
        _weicheS.setSpeed( Dcc.getCV( _cvAdr+PAR3 ) );
    }
    _pinMode( _outP[1], OUTPUT );
    _pinMode( _outP[2], OUTPUT );
    // Servowerte und Relaisausgang initiieren und ausgeben
    if ( Dcc.getCV( _cvAdr+STATE ) == GERADE ) {
        _weicheS.write( Dcc.getCV( _cvAdr+PAR1 ) );
    } else {
        _weicheS.write( Dcc.getCV( _cvAdr+PAR2 ) );
    }
    _fktStatus = Dcc.getCV( _cvAdr+STATE );
    _relOut = _fktStatus;
    _digitalWrite( _outP[REL1P], _relOut );
    _digitalWrite( _outP[REL2P], !_relOut );
   
}

void Fservo::chkState( uint8_t *sollWert ) {
    // Prüfen auf Weichenverstellung / Umstellvorgang kontrollieren
    // Diese Methode muss in jedem loop() Durchlauf aufgerufen werden

    if ( _fktStatus & MOVING ) {
        // Weiche wird gerade ungestellt, Schaltpunkt Relais und Bewegungsende überwachen
        if ( *sollWert != (_fktStatus & 0x01) && (Dcc.getCV(_cvAdr+MODE) & SDIRECT) ) {
            // Es wurde die Servoposition umgeschalten und das Flag SDIRECT ist
            // gesetzt: Bewegung abbrechen und Moving-Bit löschen.
            // Im nächsten loop-Durchlauf wird dann auf die neue Position reagiert
            _weicheS.write( _weicheS.read() );
            _fktStatus &= 0x1; 
        }
        if ( _weicheS.moving() < 50 ) _relOut = _fktStatus& 0x1;
        if ( _weicheS.moving() == 0 ) {
            // Bewegung abgeschlossen, 'MOVING'-Bit löschen und Lage in CV speichern
            _fktStatus &= 0x1; 
            Dcc.setCV( _cvAdr+STATE, _fktStatus );
            if ( Dcc.getCV( _cvAdr+MODE ) & NOPOSCHK ) {
                // Soll auf 'ungültig' stellen, damit auch neue Telegramme mit gleicher
                // Position erkannt werden (ausser es wurde schon verändert )
                if ( *sollWert == _fktStatus ) *sollWert = SOLL_INVALID;
                DB_PRINT( "dccSoll=%d", *sollWert );
            }
        }
    } else if ( *sollWert != SOLL_INVALID  && (*sollWert != _fktStatus || (Dcc.getCV(_cvAdr+MODE) & NOPOSCHK))  ) {
        // Weiche muss umgestellt werden
        DB_PRINT( "Weiche stellen, Ist=%d,Soll=%d", _fktStatus, *sollWert );
        _fktStatus = *sollWert | MOVING; // Istwert auf Sollwert und MOVING-Bit setzen.
        if ( *sollWert == GERADE ) {
            _weicheS.write( Dcc.getCV(_cvAdr+PAR1) );
        } else {
            _weicheS.write( Dcc.getCV(_cvAdr+PAR2) );
        }
    }
    // Relaisausgänge setzen
    if ( _outP[REL2P] == NC ) {
        // Variante mit einem Relais, wird in Bewegungsmitte umgeschaltet
        _digitalWrite( _outP[REL1P], _relOut );
    } else {
        // Variante mit 2 Relais, während der Bewegung beide Relais abschalten
        if ( _fktStatus & MOVING ) {
            _digitalWrite( _outP[REL1P], OFF );
            _digitalWrite( _outP[REL2P], OFF );
        } else {
            // im Stillstand des Servos entsprechend relaisout schalten
            _digitalWrite( _outP[REL1P], _relOut );
            _digitalWrite( _outP[REL2P], !_relOut );
        }
    }
    
   
}

bool Fservo::isMoving () {
    // Abfrage ob servo in Bewegung
    return _weicheS.moving() > 0;
}

uint8_t Fservo::getPos(){
    // aktuelle Position des Servos ermitteln
    return _fktStatus;
}
void Fservo::adjust( uint8_t mode, uint8_t value ) {
    // Servoparameter ändern
    switch ( mode ) {
      case ADJPOSEND:
        // Justierungswert im CV der aktuellen Position speichern
        if ( _fktStatus&1 == GERADE ) Dcc.setCV( _cvAdr+PAR1, value );
        if ( _fktStatus&1 == ABZW ) Dcc.setCV( _cvAdr+PAR2, value );
        // kein break, da weichenservo auch noch auf diese Position gestellt wird.    
      case ADJPOS:
        _weicheS.write( value );
      case ADJSPEED:
        Dcc.setCV( _cvAdr+PAR3, value );
        _weicheS.setSpeed( value );
      break;
    }
}

void Fservo::center( uint8_t mode ){ 
    // Servo auf Mittelstellung bringen
    if ( mode == ABSOLUT) {
        // absolute Mittelstellung (90°)
        _weicheS.write(90);
    } else if ( mode == RELATIVE ) {
        _weicheS.write( Dcc.getCV(_cvAdr+PAR1)/2 + Dcc.getCV(_cvAdr+PAR2)/2 );
    }
}
    
 /* 
 // Konstruktor 
// Funktionalität 
*/


 


//---------------------------- FSIGNAL --------------------------------------------
//const byte SIGMODE=0, IOSTATE0, IOSTATE1, IX_VORSIG, DARKSTATES, HSOFT1, IOSTATE2, IOSTATE3, 
Fsignal::Fsignal( int cvAdr, uint8_t pins[], uint8_t adrAnz, Fsignal** vorSig ){
    // Konstruktor für den Signaldecoder mit 1 bis 3 Adressen
    // Bei Signalen steht Gesamtzustand ( 0..6 ) des Signals in dccSoll der Grundadresse
    // Signale werden immer mit dem Grundzustand initiiert ( = HP0 oder Hp00 )
    
    _cvAdr  = cvAdr;
    _adrAnz = adrAnz;
    _vorSig = vorSig;   // == NULL wenn kein Vorsignal am Mast
    // Zahl der zugeordneten Ausgangsports (maximal 8 genutzt)
    byte _outMax = min( 8, PPWA * adrAnz );   
    _outP = pins;
    _sigLed = new SoftLed*[_outMax] ;
    _fktStatus.sigBild=0x7; // ungültiges Signalbild
    _fktStatus.state = SIG_WAIT;
    _fktStatus.dark = false;
     
    DB_PRINT( "Fsignal CV=%d, AdrAnz=%d, Pins %d", _cvAdr, _adrAnz, _outMax);
    //Modi der Ausgänge setzen ( 3 Ausgänge je Adresse ) (Soft/Hard)
    for ( byte pIx=0; pIx < PPWA*adrAnz ; pIx++ ) {
        // Modi der Ausgänge setzen
        if ( _outP[pIx] != NC ) {
            // Die CV_s verwalten die pins Adressbezogen ( 1 CV für 3 Pins )
            byte sigMode = Dcc.getCV(_cvAdr + CV_BLKLEN * (pIx/PPWA) ); // Bitcodierung harte/weiche Ledumschaltung
            // sigMode enthält bitcodiert die Info ob harte/weiche Umschaltung
            byte adIx = pIx % PPWA;   // Adressbezogener Pin-Index ( 0...2 )
           //DB_PRINT( "SigMode=%02x, Index= %d, pin=%d, ", sigMode, sigO,outPin);
            if ( sigMode & (1<<adIx) ) {
                // Bit gesetzt -> harte Umschaltung
                _pinMode(_outP[pIx], OUTPUT );
                _sigLed[pIx] = NULL;
            } else {
                // Bit = 0 -> Softled
                byte att, rise, writ; // nur für Testzwecke ( DB_PRINT )
                _sigLed[pIx] = new SoftLed;
                att=_sigLed[pIx]->attach( _outP[pIx] , Dcc.getCV(_cvAdr+MODE) & LEDINVERT );
                _sigLed[pIx]->riseTime( SIG_RISETIME );
                _sigLed[pIx]->write( OFF, BULB );
                DB_PRINT( "Softled, pin %d, Att=%d", _outP[pIx], att );
            }
            //DB_PRINT( "portTyp[%d][%d] = %d" , sigO&1, wIx+(sigO>>1), portTyp[sigO&1][wIx+(sigO>>1)] );
        }
    }
    
}

//----------------------------------------------------------------------------------------
uint8_t  Fsignal::_getSigMask( uint8_t sigState ) {
    // Ausgangsmaske für Zustand sState am Signal sIx bestimmen
    // sIx; Grundindex des Signals
    // sState: Signalzustand
    static int CVBaseAdr[] = { 1,2,6,7,11,12,16,17 } ; // schon für 4 Adressen vorgesehen
    //DB_PRINT("Fsignal-Freemem %d", freeMemory() );
   return Dcc.getCV( _cvAdr + CVBaseAdr[sigState]);
}

//----------------------------------------------------------------------------------------
void Fsignal::_setSignal ( ) {
    // alle Signalausgänge entsprechend dem derzeitigen Signalzustand setzen
    //byte sigZustand; // aktueller Signalzustand, abgeleitet aus den Weichenzuständen
    byte sigOutMsk;  // Bitmaske der Ausgangsports (Bit=1:Ausgang setzen, Bit=0 Ausgang rücksetzen
                     // Diese Maske steht für jeden Signalzustand in entsprechenden CV-Paramtern:
                     // CV51+offs    Bitmuster der Ausgänge für Befehl 1.Adresse 0 (rot)
                     // CV52+offs    Bitmuster der Ausgänge für Befehl 1.Adresse 1 (grün)
                     // CV56+offs    Bitmuster der Ausgänge für Befehl 2.Adresse 0 (rot)
                     // CV57+offs    Bitmuster der Ausgänge für Befehl 2.Adresse 1 (grün)
                     // die folgenden CV's sind nur relevant bei FSIGNAL3 (3 Adressen, 8 Zustände 6 Ausgänge)
                     // CV61+offs    Bitmuster der Ausgänge für Befehl 3.Adresse 0 (rot)
                     // CV62+offs    Bitmuster der Ausgänge für Befehl 3.Adresse 1 (grün)
                     // offs= wIx*5
                     //
    if ( !_fktStatus.dark ) {
        // Signal ist nicht dunkelgeschaltet, Signalbild aufblenden
        DB_PRINT( "Sig %d EIN (%d)", _cvAdr, _fktStatus.sigBild );
        // das aktuelle Signalbild steht in _fktStatus.sigBild
        // Ausgangszustände entsprechend Signalzustand bestimmen (CV-Wert)
        sigOutMsk = _getSigMask( _fktStatus.sigBild ) ;
        // Die Ausgänge entsprechend dem aktuellen Signalbild setzen
        for ( byte i=0; i< PPWA*_adrAnz ; i++ ) {
            if ( _sigLed[i] == NULL ) {
                // Standard-Ausgang
                digitalWrite( _outP[i], sigOutMsk&1 );
            } else {
                // Softled-Ausgang
                _sigLed[i]->write( sigOutMsk&1, BULB );
            }
            sigOutMsk = sigOutMsk >> 1;
        }
        //DB_PRINT( " Signal %d, Status=0x%02x, Ausgänge: 0x%02x ", wIx, sigZustand, Dcc.getCV( CVBaseAdr[sigZustand] + CVoffs)  );
    }
}
 
//----------------------------------------------------------------------------------------
void Fsignal::_clrSignal () {
    // alle 'Soft'Leds des Signals ausschalten
    // wIx: Grundindex des Signals
    DB_PRINT( "Sig %d AUS", _cvAdr);
    // nur 'soft' Ausgangszustände löschen
    for ( byte pIx=0; pIx< PPWA*_adrAnz ; pIx++ ) {
        if ( _sigLed[pIx] != NULL ) _sigLed[pIx]->write( OFF, BULB ); 
    }
    // am Haupsignal gegebenenfalls auch das Vorsignal dunkelschalten
    if ( _vorSig != NULL  ) {
        (*_vorSig)->setDark( true );
    }
}

//-----Funktionalität--------------------------------------------------------------
void Fsignal::setDark( bool darkFlg ) {
    // Ist das Flag 'true' wird das Signal dunkelgeschaltet
    if ( darkFlg ) {
        // Signal dunkelschalten
        DB_PRINT("setDark",0);
        _clrSignal();
        _fktStatus.dark = true;
    } else {
        // Aktuelles Signalbild wieder einschalten
        _fktStatus.dark = false;
        DB_PRINT("clrDark",0);
        _setSignal();
    }
}

void Fsignal::chkState(  uint8_t *sollWert  ) {
    // Sollzustand des gesamten Signals steht in sollWert
    // nach der Bearbeitung wird sollWert auf ungültig gesetzt
    #ifdef DEBUG
    if ( *sollWert != SOLL_INVALID )
        DB_PRINT( "chkState, CV%d, Soll=%d, Ist=%d", _cvAdr, *sollWert,  _fktStatus.sigBild )
    #endif
 
    switch ( _fktStatus.state ) {
      case SIG_WAIT:  
        // warten auf Zustandsänderung am Signal
       if ( *sollWert != SOLL_INVALID && ( _fktStatus.sigBild ) != *sollWert ) {
            // Sollzustand hat sich verändert, püfen ob erlaubter Zustand
            if (  _getSigMask(*sollWert) == 0xff )  {
                // Sollzustand hat Signalmaske 0xff -> diesen Zustand ignorieren
                // Sollzustand zurücksetzen
                *sollWert = SOLL_INVALID ;
            } else {
                // Gültiger Zustand, übernehmen, Flag setzen und Timer aufziehen
                _fktStatus.sigBild = *sollWert;
                _fktStatus.state   = SIG_NEW;
                darkT.setTime( SIG_DARK_TIME ) ;
                _clrSignal(); // aktuelles Signalbild dunkelschalten
                DB_PRINT( "Sig %d neu soll= %d, ist=0x%02x ", _cvAdr, *sollWert, _fktStatus.sigBild );
                *sollWert = SOLL_INVALID;
            }
        }
        break;
      case SIG_NEW:
        // Wenn Timer abgelaufen, neues Signalbild aufschalten
        if ( ! darkT.running() ) {
            // ist abgelaufen: neues Signalbild
            _fktStatus.state = SIG_WAIT; // 
            _setSignal(); // aktuelles Signalbild einschalten
            // Dunkelschaltung am Vorsignal setzen (nur bei Hauptsignalen)
            if ( _vorSig != NULL  ) {
                byte darkStates = Dcc.getCV(_cvAdr+STATE );
                (*_vorSig)->setDark( darkStates & (1<< _fktStatus.sigBild) );
                //DB_PRINT( "darkStates=0x%02x, VorsigFlags= 0x%02x" , darkStates, fSig[vsIx-1].flags );
            }
        }
        break;
        default:
        ;
    }

}

/* DIY Zubehördecoder
 *
 * Klassen für die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * Für jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 * Jeder Klasse muss bei der Instanziierung die Basis CV-Adresse des Blocks mit den Konfigurationswerten
 * übergeben werden, sowie ein Array mit den Pin-Nummern der Ausgangsports.
 * Bei Lichtsignalen zusätzlich die Zahl der Ausgangspins (max 8) und ein Pointer auf den Objektpointer 
 * des Vorsignals am gleichen Mast (oder NULL, wenn kein Vorsignal am Mast vorhanden ist, oder es sich 
 * schon um ein Vorsignal handelt )
 * Alle Klassen haben mindestens 2 öffentliche Methoden:
 * 'process' bearbeitet interne Abläufe und muss im loop regelmäßig aufgerufen werden
 * 'set'     gibt einen Schaltbefehl an das Objekt ( z.B. Weiche links/rechts )
 * weitere Methoden sind klassenspezifisch.
 */

#include "FuncClasses.h"


//####################  allgemeine Hilfsfunktionen ##############################
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode ) {
#else
void _pinMode( byte port, byte mode ) {
#endif
    if ( port != NC ) pinMode( port,  mode );
}

void _digitalWrite( byte port, byte state ) {
    if( port != NC ) {
		digitalWrite( port, state );
		//DBSV_PRINT("DW: Pin%d=%d", port,state );
	}
}
//#########################  Klassendefinitionen #################################

//----------------------- FCOIL ---------------------------------------------
// Ansteuerung von Doppelspulenantrieben

Fcoil::Fcoil( int cvAdr, uint8_t out1P[] ) {
    // Konstruktor für Doppelspulenantriebe
    _cvAdr = cvAdr;
    _outP = out1P;
    DBCL_PRINT( "Fcoil CV=%d, OutPins %d,%d,%d ", _cvAdr, _outP[0], _outP[1], _outP[2] );
    
    for ( byte i=0; i<3; i++ ) {
        _pinMode( _outP[i], OUTPUT );
        _digitalWrite( _outP[i], LOW );
    }
	if ( getParam( MODE) & CSTATIC ) {	
		// im STATIC Mode Ausgänge setzen
		byte lastState = getParam( STATE );
		if ( getParam( MODE) & CINVERT ) {
			_digitalWrite( _outP[0], !(lastState & 1) );
			_digitalWrite( _outP[1], !(lastState & 2) );
		} else {
			_digitalWrite( _outP[0], lastState & 1 );
			_digitalWrite( _outP[1], lastState & 2 );
		}
		_flags.pulseON =0;
		_flags.sollCoil = 0;
		_flags.istCoil = 0;
		_flags.sollOut = 0;
		_flags.sollAct = 0;
	} else {
		// flags für Bearbeitung in process() setzen	
		_flags.pulseON = false;
		_flags.sollCoil = getParam( STATE )&1;
		_flags.istCoil = !_flags.sollCoil;
		_flags.sollOut = 0;
		_flags.sollAct = true;
	}
}
//..............    
void Fcoil::set( uint8_t dccSoll, uint8_t dccState ) {
    // neuen Schaltbefehl empfangen
    _flags.sollCoil = dccSoll;
    _flags.sollOut = dccState;
    _flags.sollAct = true;
	if ( getParam( MODE) & CSTATIC ) {
		// im STATIC Mode werden die Ausgänge direkt geschaltet
		if ( getParam( MODE) & CINVERT ) {
			digitalWrite( _outP[dccSoll], !dccState  );
			setState( !digitalRead(_outP[0]) | ((!digitalRead(_outP[1]))<<1) );
		} else {
			digitalWrite( _outP[dccSoll], dccState  );
			setState( digitalRead(_outP[0]) | (digitalRead(_outP[1])<<1) );
		}
	} else {
		// Schalten der Ausgänge in process()
		_flags.sollCoil = dccSoll;
		_flags.sollOut = dccState;
		_flags.sollAct = true;
	}
	// Wenn 3. Pin definiert ist, den Sollzustand ausgeben ( invertiert, wenn CINVERT Bit gesetzt ).
	_digitalWrite( _outP[2], (getParam(MODE) & CINVERT)? !_flags.sollCoil : _flags.sollCoil );
    DBCL_PRINT( "SetFcoil OutPins %d,%d,%d ", digitalRead(_outP[0]), digitalRead(_outP[1]), digitalRead(_outP[2]) );
}

//..............    
void Fcoil::process() {
	// In STATIC mode keine Bearbeitung per Timer, EIN/AUS direkt in set-Methode
	if ( getParam( MODE) & CSTATIC) return;
	
        // Pulseausgägne einschalten
    if (  !_flags.pulseON && !_pulseT.running() &&  _flags.sollAct ) {
        // Aktionen am Ausgang nur wenn kein aktiver Impuls und der Pausentimer nicht läuft
        DBCL_PRINT(" Ist=%d, Soll=%d ", _flags.istCoil, _flags.sollCoil );
        if ( ( _flags.istCoil != _flags.sollCoil || (getParam( MODE) & NOPOSCHK) )
                && _flags.sollOut ) {
            // Weiche soll geschaltet werden
            DBCL_PRINT(" State=%d",  _flags.sollOut );
            if ( (_flags.sollCoil & 1) == 0 ) {
                // Out1 aktiv setzen
                _digitalWrite( _outP[0], HIGH );
                if ( _outP[0] != _outP[1] ) _digitalWrite( _outP[1], LOW );
                //DBCL_PRINT( "Pin%d HIGH, Pin%d LOW", _outP[0], _outP[1] );
            } else {
                // Out2 aktiv setzen
                _digitalWrite( _outP[1], HIGH );
                if ( _outP[0] != _outP[1] ) _digitalWrite( _outP[0], LOW );
                //DBCL_PRINT( "Pin%d LOW, Pin%d HIGH", _outP[0], _outP[1] );
            }
            _flags.pulseON = true;
            if ( getParam( PAR1) > 0 ) _pulseT.setTime( getParam( PAR1) * 10 );
            _flags.istCoil = _flags.sollCoil;
            setState( _flags.sollCoil );
        }
        _flags.sollAct = false; // Sollwert wurde bearbeitet.
    }
    
    // Pulsausgänge ausschalten
    if ( _flags.pulseON ) {
        // prüfen ab Impuls abgeschaltet werden muss
        if ( !(getParam( MODE) & CAUTOOFF) && _flags.sollAct && ! _flags.sollOut ) {
            // ein Abschalttelegramm wurde empfangen
            _flags.pulseON = false;
            _pulseT.setTime( 0 );     // Timer abschalten, falls er läuft
            _flags.sollAct = false;  // Empfangenes Telegramm wurde bearbeitet.
        }
        // Timerabschaltung
        if ( (getParam( PAR1) > 0) && ! _pulseT.running()  ) {
            //  Timerabschaltung ist aktiv und Timer ist abgelaufen
            //DBCL_PRINT( "Pin%d LOW, Pin%d LOW", _outP[0], _outP[1] );
            _flags.pulseON = false;
        }
        
        if ( _flags.pulseON == false ) {
            _digitalWrite( _outP[0], LOW );
            _digitalWrite( _outP[1], LOW );
            // Timer für Pulspause setzen
            if ( getParam( PAR2) > 0 ) _pulseT.setTime( getParam( PAR2) * 10 );
        }
        
    }

}


//------------------------FSTATIC -------------------------------------------- 
// Ansteuerung von statischen/blinkenden Leds

Fstatic::Fstatic( int cvAdr, uint8_t ledP[], bool extended ) {
    // Konstruktor der Klasse für statisches Leuchten bzw. blinken
	byte modeOffs = 0;	// Im normalen Mode gibt es nur ein Modebyte
    _cvAdr = cvAdr;
    _ledP = ledP;
	_flags.extended = extended;
	if ( extended ) {
		modeOffs = 3;	// Im extended Mode hat jeder Pin sein eigenes Mode-Byte
	}

    DBST_PRINT( "Fstatic,%d CV=%d, LedPins %d,%d, %d ", extended, _cvAdr, _ledP[0], _ledP[1], _ledP[2] );
    // Ausgangsports einrichten
	for ( byte pNr=0; pNr < 3; pNr++ ) {
		byte modeIx = MODE+(pNr*modeOffs);
		// Der 3.Pin kann nur im extended mode ein SoftLed sein
		if ( (getParam( modeIx ) & BLKSOFT) && (extended || pNr != 2 ) ) {
        // Ausgangsports als Softleds einrichten
            if ( _ledP[pNr] != NC ) {
                _ledS[pNr] = new SoftLed;
                byte att;
                int rise;
                att=_ledS[pNr]->attach( _ledP[pNr] );
                rise = (getParam(modeIx) >> 4) * 100;
                if ( rise == 0 ) rise = 500; // defaultwert
                _ledS[pNr]->riseTime( rise );
                _ledS[pNr]->write( OFF, LINEAR );
                DBST_PRINT( "Softled, pin %d, Att=%d", _ledP[pNr], att );
            }
		} else {
        _pinMode( _ledP[pNr], OUTPUT );
        DBST_PRINT( "Hardled, pin %d ", _ledP[pNr] );
		}
	}
    // Grundstellung der Ausgangsports
    _flags.isOn = !getParam( STATE );
    _flags.blkOn = false;       // Blinken startet mit AUS
    set( getParam( STATE ) );
}

//..............    
void Fstatic::set( bool sollOn ) {
    // Funktion ein/ausschalten
    if ( sollOn != _flags.isOn ) {
		if ( _flags.extended ) {
			// extended Mode (FSTATIC3): 3 Pins aus/einschalten. Bei Blinkmode Timer starten
			for ( byte pNr=0; pNr<3; pNr++ ) {
				byte modeIx = MODE+(pNr*MODEOFFS);
				_setLedPin(pNr, sollOn );
				DBST_PRINT( "Soll=%d, Mode=%02x", sollOn, getParam( modeIx ) );
				if ( sollOn && (getParam( modeIx ) & BLKMODE) && getParam( modeIx+1) > 0 ) {
					// Einschalten und Blinkmodus: Timer starten
					_pulseT[pNr].setTime( getParam( modeIx+1)*10 );
					DBST_PRINT( "BlinkTime %d = %d", pNr, getParam( modeIx+1)*10 );
				}
			}
		} else {
			// normaler Mode ( FSTATIC )
			_digitalWrite( _ledP[2], sollOn );	// gegebenenfalls 3.Pin statisch schalten
			_setLedPin(0, sollOn );
			if ( getParam( MODE) & BLKMODE ) {
				_setLedPin(1, (getParam( MODE ) & BLKSTRT)&& sollOn ); 
			} else {                   
				_setLedPin(1, !sollOn );                    
			}
			DBST_PRINT( "Fkt=%d, Soll=%d, Ist=%d", _cvAdr, sollOn, _flags.isOn );
			if ( sollOn && ( getParam( MODE ) & BLKMODE ) ) {
				// Funktion wird eingeschaltet und Blinkmode ist aktiv -> Timer setzen
				_pulseT[0].setTime( getParam( PAR3)*10 );
				DBST_PRINT( "BlkEin %d/%d, Strt=%x", getParam( PAR1) , getParam( PAR2), (getParam( MODE) & BLKSTRT)  );
				_flags.blkOn = true;
			}
		}
		_flags.isOn = sollOn;
		setState( _flags.isOn );
	}
}

//..............    
void Fstatic::process( ) {
    // Blinken der Leds steuern
    if ( _flags.isOn ) {
		// nur im aktiven Status wird geblinkt
	    if ( _flags.extended ) {
			for ( byte ledI=0; ledI<3; ledI++ ) {
				byte modeIx = MODE+(ledI*MODEOFFS);
				if ( (getParam( modeIx ) & BLKMODE) && getParam( modeIx+1 ) > 0 && !_pulseT[ledI].running() ) {
					// Timer abgelaufen, Led-Status wechseln
					if ( bitRead( _pinStat, ledI ) ) {
						// Led ausschalten
						_setLedPin(ledI, LOW );
						if ( getParam( modeIx+2 ) > 0 ) _pulseT[ledI].setTime( getParam( modeIx+2 )*10 );
						else _pulseT[ledI].setTime( getParam( modeIx+1 )*10 );
					} else { 
						// Led einschalten
						_setLedPin(ledI, HIGH );
						_pulseT[ledI].setTime( getParam( modeIx+1 )*10 );
					}
				}
			}
		} else if ( getParam( MODE ) & BLKMODE ) {
			// bei aktivem Blinken die Timer abfragen/setzen
			if ( !_pulseT[0].running() ) {
				// Timer abgelaufen, Led-Status wechseln
				if ( _flags.blkOn ) {
					// Led ausschalten
					_setLedPin(0, LOW );
					_setLedPin(1, HIGH );
					_flags.blkOn = false;
					_pulseT[0].setTime( getParam( PAR2 )*10 );
				} else {
					// Led einschalten
					_setLedPin(0, HIGH );
					_setLedPin(1, LOW );
					_flags.blkOn = true;
					_pulseT[0].setTime( getParam( PAR1 )*10 );
				}
			}
		}
    }
}

//..............    
void Fstatic::_setLedPin( uint8_t ledI, bool sollWert ) {
    // den LED ausgang setzen. je nach Konfiguration als Softled oder hart
	bool outState = sollWert;
    if ( ledI < 3 ) {
		// Im extended Mode Ausgabewert gegebenenfalls invertieren
		if ( _flags.extended && ( FSTAINV & getParam( MODE+(MODEOFFS*ledI) ) ) ) outState = !outState; 
        if ( _ledS[ledI] != NULL ) _ledS[ledI]->write( outState);
        else _digitalWrite( _ledP[ledI], outState );
		bitWrite( _pinStat, ledI, sollWert );
		DBST_PRINT( "Pin: %d, Val: %d, pStat=%02x", _ledP[ledI], outState, _pinStat );
    }
}

//----------------------------- FSERVO --------------------------------------------
// Ansteuerung von Servo-Antrieben
// offset für die CV's der Positionswerte
const uint8_t Fservo::posOffset[6]={PAR1,PAR2,PAR1+CV_BLKLEN,PAR2+CV_BLKLEN,PAR1+2*CV_BLKLEN,PAR2+2*CV_BLKLEN } ;   
const uint8_t _relIx[] = {1,2,4,5}; //Index der relais-Pins im Pin-Array   

Fservo::Fservo( int cvAdr, uint8_t pins[], uint8_t posZahl, int8_t modeOffs ) {
    // Konstruktor der ServoKlasse
    _outP = pins;
    _posZahl = posZahl;
    _cvAdr = cvAdr;
    _modeOffs=modeOffs;
    _parOffs=0;
    if ( modeOffs > 0 ) {
        _parOffs = modeOffs;    // Ist 2. Servo auf primärer Adresse (F2SERVO) 
    }
    // Weichenservo einrichten
    if ( _outP[SERVOP] != NC ) {
        _weicheS.attach( _outP[SERVOP], getParam( _modeOffs ) & SAUTOOFF );
        _weicheS.setSpeed( getParam( PAR3+_parOffs ) );
    }
    for ( uint8_t i = 0; i<posZahl; i++ ) {
        _pinMode( _outP[_relIx[i]], OUTPUT );
        _digitalWrite( _outP[_relIx[i]], LOW );      // notwendig für STM32/PA15 ( dieser Port ist HIGH nach pinMode )
    }
    // Servowerte und Relaisausgang initiieren und ausgeben
    if ( getParam(_modeOffs) & SAUTOBACK )  _istPos = 0;
    else                                    _istPos = getParam( STATE );
    _sollPos = _istPos ;
    _flags.relOn = _istPos;
	// 11.8.23 ( V7.1.2) Relais werden imer in .process geschaltet
    /*if ( posZahl > 2 ) {
        // 4-Position-Servo: Relais immer entsprechend aktueller Position setzen
        _digitalWrite( _outP[_relIx[_istPos]], ON );
    } else {
        // Servo mit 2 Positionen ( Mittenumschaltung oder 2 Relais mit Positions-Schaltung )
        _digitalWrite( _outP[REL1P], _flags.relOn );
        _digitalWrite( _outP[REL2P], !_flags.relOn );
    }*/
    _flags.sollAct = false;
    _flags.moving = false;
    _weicheS.write( getParam( posOffset[_istPos]+_parOffs ) );
    DBSV_PRINT("ServoObj@%04x, Pins=%d %d %d %d %d %d , cvAdr=%d, modeOffs=%d", (uint32_t)this , _outP[0],_outP[1],_outP[2],_outP[3],_outP[4],_outP[5], _cvAdr, _modeOffs);
    DBSV_PRINT("ModeByte=%02x", getParam( _modeOffs ) ) ;
   
}

//..............    
void Fservo::set( uint8_t newPos ) {
    // Befehl 'servo stellen' erhalten
    if ( newPos >= _posZahl ) newPos = _posZahl-1;   // maximalZhal der Positionswerte
    _sollPos = newPos;
    DBSV_PRINT( "Servo.set(%d): new:%d, soll:%d, max:%d", _outP[0], newPos, _sollPos, _posZahl );
    _flags.sollAct = true;
}
//..............    
void Fservo::process() {
    // Umstellvorgang kontrollieren
    // Diese Methode muss in jedem loop() Durchlauf aufgerufen werden
    if ( _autoTime.expired() && _sollPos ) {
        // Servo steht in Arbeitsstellung und Zeit ist abgelaufen: zurückfahren
        _sollPos = 0;
        _flags.sollAct = true;
        DBSV_PRINT( "(%04lx) ServoTimer abgelaufen, _istlAbz=%d",(uint32_t)this, _istPos  );
     }   
    
    if ( _flags.moving ) {
        // Weiche wird gerade ungestellt, Schaltpunkt Relais und Bewegungsende überwachen
        if ( _sollPos != _istPos && (getParam( _modeOffs) & SDIRECT) ) {
            // Es wurde die Servoposition umgeschalten und das Flag SDIRECT ist
            // gesetzt: Bewegung abbrechen und Moving-Bit löschen.
            // Im nächsten loop-Durchlauf wird dann auf die neue Position reagiert
            _weicheS.write( _weicheS.read() );
            _flags.moving = false;; 
        }
        if ( _weicheS.moving() < 50 ) _flags.relOn = _istPos;
        if ( _weicheS.moving() == 0 ) {
            // Bewegung abgeschlossen, 'MOVING'-Bit löschen und Lage in CV speichern
            _flags.moving = false; 
            _flags.sollAct = false;
            if ( getParam( _modeOffs ) & SAUTOBACK ) {
                if ( _istPos ) {
                    // Bei Arbeitsstellung Timer für Rückfahren starten
                    if ( getParam(PAR4+_parOffs) <= 1 ) _autoTime.setTime(SAUTOTIME);
                    else                        _autoTime.setTime( getParam(PAR4+_parOffs) * 100 );
                    DBSV_PRINT("(%04lx) Timer gestartet, Zeit=%d",(uint32_t)this, _autoTime.getTime() );
                }
            } else {
                // ohne Autoback aktuelle Lage speicern
                setState( _istPos );
            }
            /*if ( getParam( _modeOffs ) & NOPOSCHK ) {
                // Soll auf 'ungültig' stellen, damit auch neue Telegramme mit gleicher
                // Position erkannt werden (ausser es wurde schon verändert )
                if ( _sollPos == _fktStatus ) _sollPos = SOLL_INVALID;
                DBSV_PRINT( "dccSoll=%d", _sollPos );
            }*/
        }
    } else if ( _flags.sollAct  && (_sollPos != _istPos || (getParam( _modeOffs) & NOPOSCHK))  ) {
        // Weiche muss umgestellt werden
        _istPos = _sollPos;    // Istwert auf Sollwert 
        _flags.moving = true;               // und MOVING-Flagt setzen.
        DBSV_PRINT("Servo-write=%d", getParam( posOffset[_sollPos]+_parOffs ) );
        _weicheS.write( getParam( posOffset[_sollPos]+_parOffs ) );
    }
    // Relaisausgänge setzen
    if ( _outP[REL2P] == NC && _posZahl == 2  ) {
        // Variante mit einem Relais, wird in Bewegungsmitte umgeschaltet
        _digitalWrite( _outP[REL1P], _flags.relOn );
    } else {
        // Variante mit 2 oder 4 Relais, während der Bewegung alle Relais abschalten
        if ( _flags.moving ) {
            for ( byte i=0; i<_posZahl; i++ ) {
                _digitalWrite( _outP[_relIx[i]], OFF );
            }
        } else {
            // im Stillstand des Servos entsprechend relaisout schalten
            _digitalWrite( _outP[_relIx[_sollPos]], ON );
        }
    }
}
//..............    
bool Fservo::isMoving () {
    // Abfrage ob servo in Bewegung
    return _weicheS.moving() > 0;
}
//..............    
uint8_t Fservo::getPos(){
    // aktuelle Position des Servos ermitteln
    return _istPos;
}
//..............    
uint8_t Fservo::getCvPos(){
    // CV Wert für aktuelle Position des Servos ermitteln
    return getParam( posOffset[_sollPos]+_parOffs );
}
//..............    
void Fservo::adjust( uint8_t mode, uint8_t value ) {
    // Servoparameter ändern
    DBSV_PRINT( "adjust: mode=%d, val=%d", mode, value );
    switch ( mode ) {
      case ADJPOSEND:
        // Justierungswert im CV der aktuellen Position speichern
        setParam( posOffset[_istPos]+_parOffs, value );
        // kein break, da weichenservo auch noch auf diese Position gestellt wird. 
        [[fallthrough]];
      case ADJPOS:
        _weicheS.write( value );
        break;
      case ADJSPEED:
        setParam( PAR3+_parOffs, value );
        _weicheS.setSpeed( value );
        DBSV_PRINT("AdjSpeed, %04X = %d ( CV %d ) ", (uint16_t)this, value, PAR3+_parOffs ); 
        break;
    }
}
//..............    
void Fservo::center( uint8_t mode ){ 
    // Servo auf Mittelstellung bringen
    if ( mode == ABSOLUT) {
        // absolute Mittelstellung (90°)
        _weicheS.write(90);
    } else if ( mode == RELATIVE ) {
        _weicheS.write( getParam( PAR1)/2 + getParam( PAR2)/2 );
    }
}
    


//---------------------------- FSIGNAL --------------------------------------------
// Ansteuerung von Lichtsignalen

Fsignal::Fsignal( int cvAdr, uint8_t pins[], uint8_t pinAnz, Fsignal** vorSig ){
    // Konstruktor für den Signaldecoder mit 1 bis 3 Adressen
    // Signale werden immer mit dem Grundzustand initiiert ( = HP0 oder Hp00 )
    
    _cvAdr  = cvAdr;
    _pinAnz = min( 8, pinAnz );
    _vorSig = vorSig;   // == NULL wenn kein Vorsignal am Mast
    // Zahl der zugeordneten Ausgangsports (maximal 8 genutzt)
    _outP = pins;
    _sigLed = new SoftLed*[_pinAnz] ;
    _fktStatus.sigBild=0x7; // ungültiges Signalbild
    _fktStatus.state = SIG_WAIT;
    _fktStatus.dark = false;
     
    DBSG_PRINT( "Fsignal CV=%d, Pins %d", _cvAdr, _pinAnz);
    //Modi der Ausgänge setzen ( 3 Ausgänge je Adresse ) (Soft/Hard)
    for ( byte pIx=0; pIx < _pinAnz ; pIx++ ) {
        // Modi der Ausgänge setzen
        byte sigMode = _getHsMask(); // Bitcodierung harte/weiche Ledumschaltung
		_sigLed[pIx] = NULL;		 // Softled-Pointer mit NULL initiieren
        if ( _outP[pIx] != NC ) {
            // Die CV_s verwalten die pins Adressbezogen ( 1 CV für 3 Pins )
            // sigMode enthält bitcodiert die Info ob harte/weiche Umschaltung
           //DBSG_PRINT( "SigMode=%02x, Index= %d, pin=%d, ", sigMode, sigO,outPin);
            if ( sigMode & (1<<pIx) ) {
                // Bit gesetzt -> harte Umschaltung
                _pinMode(_outP[pIx], OUTPUT );
                _sigLed[pIx] = NULL;
            } else {
                // Bit = 0 -> Softled
                byte att; // nur für Testzwecke ( DBSG_PRINT )
                _sigLed[pIx] = new SoftLed;
                att=_sigLed[pIx]->attach( _outP[pIx] , getParam( LSMODE ) & LEDINVERT );
                _sigLed[pIx]->riseTime( SIG_RISETIME );
                _sigLed[pIx]->write( OFF, BULB );
                DBSG_PRINT( "Softled, pin %d, Att=%d", _outP[pIx], att );
            }
            //DBSG_PRINT( "portTyp[%d][%d] = %d" , sigO&1, wIx+(sigO>>1), portTyp[sigO&1][wIx+(sigO>>1)] );
		}
    }
    _fktStatus.sigBild = 1;
    set( 0 );
    DBSG_PRINT( "Konstruktor %d", _cvAdr );
}


//----- öffentliche Methoden -----------------------------------------------------------
// Signal dunkelschalten ( genutzt für Vorsignale am Mast eines Hauptsignals )
void Fsignal::setDark( bool darkFlg ) {
    // Ist das Flag 'true' wird das Signal dunkelgeschaltet
    if ( darkFlg ) {
        // Signal dunkelschalten
        DBSG_PRINT("setDark");
        _clrSignal(0);
        _fktStatus.dark = true;
    } else {
        // Aktuelles Signalbild wieder einschalten
        _fktStatus.dark = false;
        DBSG_PRINT("clrDark");
        _setSignal();
    }
}

//..............    
// Signalbild umschalten
void Fsignal::set( uint8_t sollWert ) {
    DBSG_PRINT( "setSignal, CV%d, Soll=%d, Ist=%d", _cvAdr, sollWert,  _fktStatus.sigBild );
    if (  _fktStatus.sigBild != sollWert ) {
        // Sollzustand hat sich verändert, püfen ob erlaubter Zustand
        if (  _getSigMask( sollWert) == 0xff )  {
            // Sollzustand hat Signalmaske 0xff -> diesen Zustand ignorieren
            // Sollzustand zurücksetzen
            DBSG_PRINT("SigMask(soll) = %02X", _getSigMask( sollWert) );
        } else {
            // Gültiger Zustand, übernehmen, Flag setzen und Timer aufziehen
            _fktStatus.sigBild =  sollWert;
            _fktStatus.state   = SIG_NEW;
            _getSigMask( _fktStatus.sigBild ) ;
            DBSG_PRINT( "SigMSk Stat=%02X, Blnk=%02X, Inv=%02X", _sigMask.staticLed, _sigMask.blnkStd, _sigMask.blnkInv );
            darkT.setTime( SIG_DARK_TIME ) ;
            // im aktuellen Signalbild alles dunkelschalten, was im neuen Signalbild nicht leuchtet
            _clrSignal(_sigMask.staticLed|_sigMask.blnkStd|_sigMask.blnkInv); 
            //_clrSignal(_sigMask.staticLed); 
            DBSG_PRINT("Ende set %d", _cvAdr );
        }
    }
    
}
//..............    
// Umschalten des Signalbilds steuern ( muss im loop() aufgerufen werden )
void Fsignal::process() {
 
    switch ( _fktStatus.state ) {
      case SIG_WAIT:  
        // warten auf Zustandsänderung am Signal
        // wurde 'set' aufgerufen, und das Signalbild muss umgeschaltet werden, so 
        // wird die Statemachine in der 'set' Methode weitergeschaltet
        
        // bei blinkenden Signalbildern die Blinkenden Leds umschalten
        if ( _blinkT.expired() ) {
            _fktStatus.blinkTakt = !_fktStatus.blinkTakt;
            _setSignalBlink ( );
        }
        break;
      case SIG_NEW:
        // Wenn Timer abgelaufen, neues Signalbild aufschalten
        if ( ! darkT.running() ) {
            // ist abgelaufen: neues Signalbild
            _fktStatus.state = SIG_WAIT; // 
            _setSignal(); // aktuelles Signalbild einschalten
           
            // Dunkelschaltung am Vorsignal setzen (nur bei Hauptsignalen)
            if ( _vorSig != NULL && *_vorSig != NULL  ) {
                byte darkStates = getParam( DARKMASK );
                (*_vorSig)->setDark( darkStates & (1<< _fktStatus.sigBild) );
                DBSG_PRINT( "darkStates=0x%02x, Signalbild=%d" , darkStates, _fktStatus.sigBild );
            }
        }
        break;
        default:
        ;
    }

}

//-------------- private Methoden -----------------------------------------------
// Maske Hard/Soft für alle Pins bestimmen 
uint8_t Fsignal::_getHsMask(){
    // die Bits stehen jeweils in 3er Gruppen bei den Parametern je Adresse
    // Maximal können 8 Ausgänge verwaltet werden
    uint8_t hsMask = getParam( LSMODE ) & 0x7;
    if ( _pinAnz > PPWA ) hsMask |= (getParam( SOFTMASK2 ) & 0x7) << 3; 
    if ( _pinAnz > 2*PPWA ) hsMask |= (getParam( SOFTMASK3 ) & 0x3) << 6;     
    return hsMask;
}

//..............    
// Ausgangsmasken für Signalbild sigState bestimmen
byte  Fsignal::_getSigMask( uint8_t sigState ) {
	// Zurückgegeben wird die statische Maske ( für den Test auf 0xFF um Kommandos zu ignorieren )
    // sState: Signalzustand
    //static int parOffs[] = { 1,2,6,7,11,12,16,17 } ; // max 4 Adressen vorgesehen
    byte parOffs = ((sigState>>1) * CV_BLKLEN ) + (sigState&1)  +1;
    // der Offset für die Blinkparameter ist um BLINK1-BILD1 größer
    byte staticMask = getParam( parOffs );
    byte blinkMask = getParam( parOffs+(BLINK1-BILD1) );
    DBSG_PRINT("stMsk=%02X, blMsk=%02X ( Ofs=%d, Ofb=%d )", staticMask, blinkMask, parOffs, parOffs+(BLINK1-BILD1) );
    //DBSG_PRINT("Fsignal-Freemem %d", freeMemory() );
    _sigMask.staticLed =  staticMask & ~blinkMask;  // nur Bit in staticMask gesetzt
    _sigMask.blnkStd =  blinkMask & ~staticMask; // nur Bit in blnkMask gesetzt
    _sigMask.blnkInv =  staticMask & blinkMask;  // Bit in beiden Masken gesetzt
    DBSG_PRINT("  ..static=%02X, blk=%02X, inv=%02X", _sigMask.staticLed, _sigMask.blnkStd, _sigMask.blnkInv );
   return staticMask;
}

//.............. 
// neues Signalbild initiieren
void    Fsignal::_setSignal ( ) {
            _fktStatus.blinkTakt = true;
            _setSignalStatic(); // aktuelles Signalbild einschalten / statische Leds
            _setSignalBlink();  // aktuelles Signalbild einschalten / blinkende Leds
}

// alle statischen Signalausgänge entsprechend dem derzeitigen Signalzustand setzen
void Fsignal::_setSignalStatic ( ) {
    //byte sigZustand; // aktueller Signalzustand, abgeleitet aus den Weichenzuständen
    byte sigOutMsk;  // Bitmaske der Ausgangszustände (Bit=1:Ausgang setzen, Bit=0 Ausgang rücksetzen
    byte sigStaMsk;  // Bitmaske der statischen Leds ( nur diese werden geschaltet )
    if ( !_fktStatus.dark ) {
        // Signal ist nicht dunkelgeschaltet, Signalbild aufblenden
        DBSG_PRINT( "Sig %d EIN (%d)", _cvAdr, _fktStatus.sigBild );
        // das aktuelle Signalbild steht in _fktStatus.sigBild
        // Ausgangszustände entsprechend Signalzustand bestimmen (CV-Wert)
        _getSigMask( _fktStatus.sigBild ) ;
        sigOutMsk = _sigMask.staticLed ;
        sigStaMsk = ~(_sigMask.blnkInv | _sigMask.blnkStd) ;
        // Die Ausgänge entsprechend dem aktuellen Signalbild setzen
        for ( byte i=0; i< _pinAnz ; i++ ) {
            if ( sigStaMsk&1 ) {
                // Es ist eine statisch zu schaltende Led
                if ( _sigLed[i] == NULL ) {
                    // Standard-Ausgang
                    _digitalWrite( _outP[i], sigOutMsk&1 );
                } else {
                    // Softled-Ausgang
                    DBSG_PRINT(" Static-LED%d=%d", i,sigOutMsk&1 );
                    _sigLed[i]->write( sigOutMsk&1, LINEAR );
                }
            }
            sigOutMsk = sigOutMsk >> 1;
            sigStaMsk = sigStaMsk >> 1;
        }
        //DBSG_PRINT( " Signal %d, Status=0x%02x, Ausgänge: 0x%02x ", wIx, sigZustand, Dcc.getCV( CVBaseAdr[sigZustand] + CVoffs)  );
    }
}
 
// alle blinkenden Signalausgänge entsprechend dem derzeitigen Signalzustand und Taktphase setzen
void Fsignal::_setSignalBlink ( ) {
    //byte sigZustand; // aktueller Signalzustand, abgeleitet aus den Weichenzuständen
    byte sigOutMask;   // Bitmaske aller blinkenden Leds
    byte sigOutMskInv;  // Bitmaske der Ausgangsports (Bit=1:Ausgang blinkt )
    if ( !_fktStatus.dark ) {
        // Signal ist nicht dunkelgeschaltet, Signalbild aufblenden
        // das aktuelle Signalbild steht in _fktStatus.sigBild
        // Ausgangszustände entsprechend Signalzustand bestimmen (CV-Wert)
        _getSigMask( _fktStatus.sigBild ) ;
        sigOutMask   = _sigMask.blnkInv | _sigMask.blnkStd ;
        sigOutMskInv = _sigMask.blnkInv ;
        DBSG_PRINT( "Blinktakt MSK=%02X, INV=%02X", sigOutMask, sigOutMskInv );
        if ( sigOutMask ) {
            // es gibt blinkende Leds
            // Die Ausgänge entsprechend dem aktuellen Signalbild setzen
            for ( byte i=0; i< _pinAnz ; i++ ) {
                if ( sigOutMask&1 ) {
                    // es ist eine blinkende Led
                    if ( _sigLed[i] == NULL ) {
                        // Standard-Ausgang
                        _digitalWrite( _outP[i], _fktStatus.blinkTakt ^ (sigOutMskInv&1) );
                    } else {
                        // Softled-Ausgang
                        DBSG_PRINT(" Blink-LED%d=%d", i,_fktStatus.blinkTakt ^ (sigOutMskInv&1) );
                        _sigLed[i]->write( _fktStatus.blinkTakt ^ (sigOutMskInv&1), LINEAR );
                    }
                }
                sigOutMskInv = sigOutMskInv >> 1;
                sigOutMask = sigOutMask >> 1;
            }
            int taktZeit = getParam( _fktStatus.blinkTakt?BLINKTAKT1:BLINKTAKT2  ) *10;
            if ( taktZeit == 0 ) taktZeit = SIG_RISETIME + SIG_RISETIME/2 ; // bei fehlender Taktzeit Zeit etwas länger als Aufblendzeit der Led
            DBSG_PRINT( "Sig %d Takt= %d, Zeit=%d BT1=%d, BT2=%d", _cvAdr, _fktStatus.blinkTakt?BLINKTAKT1:BLINKTAKT2, taktZeit, getParam(BLINKTAKT1), getParam(BLINKTAKT2) );
            _blinkT.setTime(  taktZeit );
        }
        //DBSG_PRINT( " Signal %d, Status=0x%02x, Ausgänge: 0x%02x ", wIx, sigZustand, Dcc.getCV( CVBaseAdr[sigZustand] + CVoffs)  );
    }
}
 
//..............    
// alle Signallampen ausschalten ( beim Überblenden zwischen Signalbildern )
void Fsignal::_clrSignal ( byte onMsk ) {
    // 'Soft'Leds des Signals entspr onMsk ausschalten
    // Bits, die in onMsk gesetzt sind, werden nicht ausgeschaltet
    if (!(getParam( LSMODE ) & NODARKLED) ) onMsk = 0; // Immer alles dunkelschalten
    DBSG_PRINT( "Sig %d AUS", _cvAdr);
    // nur 'soft' Ausgangszustände löschen
    for ( byte pIx=0; pIx< _pinAnz ; pIx++ ) {
        if ( _sigLed[pIx] != NULL && (onMsk&1) == 0) _sigLed[pIx]->write( OFF, BULB ); 
        onMsk = onMsk >> 1;
    }
    // am Haupsignal gegebenenfalls auch das Vorsignal dunkelschalten
    if ( _vorSig != NULL && *_vorSig != NULL ) {
        DBSG_PRINT("Vorsig dunkelschalten");
        (*_vorSig)->setDark( true );
    } 
}

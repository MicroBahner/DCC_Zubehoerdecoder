// #define DEBUG ;
#include "FuncClasses.h"

extern NmraDcc Dcc;

#define GetCvPar(par) Dcc.getCV(cvAdr+par)
//----------------------- FCOIL ---------------------------------------------

Fcoil::Fcoil( int cvAdr, uint8_t out1P, uint8_t out2P ) {
    // Konstruktor für Doppelspulenantriebe
    _cvAdr = cvAdr;
    _outP[0] = out1P;
    _outP[1] = out2P;
    DB_PRINT( "Fcoil CV=%d, OutPis %d,%d ", _cvAdr, _outP[0], _outP[1] );
    
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

Fstatic::Fstatic( int cvAdr, uint8_t led1P, uint8_t led2P ) {
    // Konstruktor der Klasse für statisches Leuchten bzw. blinken
    _cvAdr = cvAdr;
    _ledP[0] = led1P;
    _ledP[1] = led2P;
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
void Fstatic::chkState( uint8_t sollWert ) {
    // muss Ausgang umgeschaltet werden?
    if ( (sollWert&1) != (_fktStatus&1) ) {
        _setLedPin(0, sollWert );
        if ( Dcc.getCV( _cvAdr+MODE) & BLKMODE ) {
            _setLedPin(1, (Dcc.getCV( _cvAdr+MODE ) & BLKSTRT)&& (sollWert&1) ); 
        } else {                   
            _setLedPin(1, !sollWert );                    
        }
        //DB_PRINT( "Soll=%d, Ist=%d", sollWert, _fktStatus );
        _fktStatus = sollWert;
        Dcc.setCV( _cvAdr+STATE, _fktStatus );
        if ( _fktStatus && ( Dcc.getCV( _cvAdr+MODE ) & BLKMODE ) ) {
            // Funktion wird eingeschaltet und Blinkmode ist aktiv -> Timer setzen
            pulseT.setTime( Dcc.getCV( _cvAdr+PAR3)*10 );
            //DB_PRINT( "BlkEin %d/%d, Strt=%x", Dcc.getCV( _cvAdr+PAR1) , Dcc.getCV( _cvAdr+PAR2), (Dcc.getCV( _cvAdr+MODE) & BLKSTRT)  );
            _fktStatus |= BLKON;
        }
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
#define DEBUG ;
#include "FuncClasses.h"

extern NmraDcc Dcc;

#define GetCvPar(par) Dcc.getCV(cvAdr+par)

Fstatic::Fstatic( uint16_t cvAdr, uint8_t led1P, uint8_t led2P ) {
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
    // Timer einrichten
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
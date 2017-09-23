/* DIY Zubehördecoder
 *
 * Klassen für die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * Für jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 */
#include "Globals.h"
// Offset der CV-Adresse bei den Funktionsspezifschen CV-Werten
const byte MODE=0, PAR1=1, PAR2=2, PAR3=3, STATE=4 ;

//======================  allgemeine Hilfsfunktionen ==================================
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode );
#else
void _pinMode( byte port, byte mode );
#endif

void _digitalWrite( byte port, byte state ) ;

//========================= Funktionsklassen  =========================================
class Fservo {
 
 };
//---------------------- FCOIL -------------------------------------------
// Flags für CV 'MODE'
#define CAUTOOFF 0x01   // Die Impulsdauer wird intern begrenzt
#define NOPOSCHK 0x08   // Die Ausgänge reagieren auch auf einen Befehl, wenn die aktuelle
                        // Postion nicht verändert wird.
 
 class Fcoil {
     public:
    Fcoil( int cvAdr, uint8_t out1, uint8_t out2 );
    void chkState( uint8_t *sollWert, uint8_t outState );
    
    private:
    EggTimer _pulseT;
    uint8_t _pulseON;            // Flag ob Pausentimer läuft
    
    uint16_t _cvAdr = 0;            // Adresse des CV-Blocks mit den Funktionsparametern
    uint8_t _outP[2];           // Pins der Ausgange
    uint8_t _fktStatus = 0;         // interner Status: rechts/links
    #define GERADE  0x0             // Bit 0
    #define ABZW    0x1             // Bit 0

 };

//------------------------FSTATIC -------------------------------------------- 
// Flags für CV 'MODE':
#define BLKMODE 0x01    // Ausgänge blinken
#define BLKSTRT 0x02    // Starten mit beide Ausgängen EIN
#define BLKSOFT 0x04    // Ausgänge als Softleds


 class Fstatic {
    // statisches oder blinkendes Ansteuern von Led's
    public:
    Fstatic( int cvAdr, uint8_t led1, uint8_t led2 );
    void chkState( uint8_t sollWert );
    
    private:
    void _setLedPin( uint8_t ledI, uint8_t sollWert );
    EggTimer pulseT;
    
    uint16_t _cvAdr;            // Adresse des CV-Blocks mit den Funktionsparametern
    SoftLed *_ledS[2] = { NULL, NULL };      // Softled-Objekte
    uint8_t _ledP[2];           // Pins der Leds
    uint8_t _fktStatus;         // interner Status: 0= aus, 1= aktiv
    const byte BLKON = 0x4 ;    // Bit 2 nur für Static, blinkende Led ist EIN
    
 };
 
 class FSignal {
 
 };

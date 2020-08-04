/* Interface des Zubehördecoders
** Alle Interface-abhängigen ( Loconet oder DCC ) Programmkomponenten werden hier
** zusammengefasst, und neutrale Aufrufe für die Funktionalitäten im Sketch zur Verfügung gestellt.
*/
#include "../Interface.h"
#include "DebugDefs.h"
#ifdef LOCONET
// --------------- Loconet-Interface -------------------------------------------------
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
#include <LocoNet.h>
typedef enum {
    CV29_EXT_ADDRESSING      = 0b00100000,	/** bit 5: "0" = one byte addressing, "1" = two byte addressing */
    CV29_OUTPUT_ADDRESS_MODE = 0b01000000,	/** bit 6: "0" = Decoder Address Mode "1" = Output Address Mode */
    CV29_ACCESSORY_DECODER   = 0b10000000,	/** bit 7: "0" = Multi-Function Decoder Mode "1" = Accessory Decoder Mode */
} CV_29_BITS;
const uint8_t cvAccDecAddressLow   = 17;
const uint8_t cvAccDecAddressHigh  = 18;
const uint8_t cvVersionId          = SV_ADDR_SW_VERSION;
const uint8_t cvManufactId         = 8;
const uint8_t cv29Config           = 29;
const uint8_t config29Value         = CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE;
const uint8_t config29AddrMode      = CV29_OUTPUT_ADDRESS_MODE;
const uint8_t manIdValue            = 13;

LocoNetSystemVariableClass sv;
lnMsg       *LnPacket;
boolean     deferredProcessingNeeded = false;


void ifc_init( uint8_t version, uint8_t progMode, uint8_t cvPomLow ) {
    // DB_PRINT("Loconet-Init %d",0);
    LocoNet.init(txPin); 
    // DB_PRINT("Loconet-Init %d",1);
    sv.init(13, 4, 1, version ); // ManufacturerId, DeviceId, ProductId, SwVersion
    sv.writeSVStorage(SV_ADDR_NODE_ID_H, sv.readSVStorage( cvPomLow+1 ) );
    sv.writeSVStorage(SV_ADDR_NODE_ID_L, sv.readSVStorage( cvPomLow ));

    sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 0x56);
    sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_L, 0x78);
    DB_PRINT( "Init LocoNet: Node-Id = %d", sv.readSVStorage(SV_ADDR_NODE_ID_H)*256+sv.readSVStorage(SV_ADDR_NODE_ID_L) );
}

void ifc_init( uint8_t cvPomLow ) {
    sv.writeSVStorage(SV_ADDR_NODE_ID_H, sv.readSVStorage( cvPomLow+1 ) );
    sv.writeSVStorage(SV_ADDR_NODE_ID_L, sv.readSVStorage( cvPomLow ));
}

void ifc_process() {
    SV_STATUS svStatus;
    static boolean  deferredProcessingNeeded = false;
    LnPacket = LocoNet.receive() ;
    //DB_PRINT("LN-Package = %04x", LnPacket);
    if( LnPacket ) {
        #ifdef DEBUG 
        #warning " LocoNet debugging aktiv"
        // First print out the packet in HEX
        Serial.print("RX: ");
        uint8_t msgLen = getLnMsgSize(LnPacket); 
        for (uint8_t x = 0; x < msgLen; x++)
        {
          uint8_t val = LnPacket->data[x];
          // Print a leading 0 if less than 16 to make 2 HEX digits
          if(val < 16)
            Serial.print('0');        
          Serial.print(val, HEX);
          Serial.print(' ');
        }
        Serial.println();
        #endif  
        
        
        // If this packet was not a Switch or Sensor Message checks for SV packet
        if(!LocoNet.processSwitchSensorMessage(LnPacket)) {      
            svStatus = sv.processMessage(LnPacket);
            deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
        }
    }
    if(deferredProcessingNeeded) {
        deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);
    }
}
uint8_t ifc_getCV( uint16_t address ) {
    return sv.readSVStorage( address );
}

void ifc_setCV( uint16_t address, uint8_t value ) {
    DB_PRINT( "setCV: Ad=%3d, Val=%3d", address, value );
    sv.writeSVStorage( address, value );
}

uint16_t ifc_getAddr(){
    return sv.readSVStorage(cvAccDecAddressLow ) + 256*sv.readSVStorage(cvAccDecAddressHigh );
}


void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) {
    ifc_notifyDccAccState( Address, Direction ? 1 : 0, Output );
}

void notifySVChanged(uint16_t Offset){
    
    ifc_notifyCVChange( Offset, sv.readSVStorage(Offset) );
}

#else
#error "Der Zubehördecoder mit LocoNet Interface läuft nicht auf dieser Platform"
#endif

#else
// --------------- DCC-Interface -------------------------------------------------

#include <NmraDcc.h>
NmraDcc Dcc;

const uint8_t cvAccDecAddressLow   = CV_ACCESSORY_DECODER_ADDRESS_LSB;
const uint8_t cvAccDecAddressHigh  = CV_ACCESSORY_DECODER_ADDRESS_MSB;
const uint8_t cvVersionId          = CV_VERSION_ID;
const uint8_t cvManufactId         = CV_MANUFACTURER_ID;
const uint8_t cv29Config           = CV_29_CONFIG;
const uint8_t config29Value         = CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE;
const uint8_t config29AddrMode      = CV29_OUTPUT_ADDRESS_MODE;
const uint8_t manIdValue            = MAN_ID_DIY;

void ifc_init( uint8_t version, uint8_t progMode, uint8_t cvPomLow ) {
    // nmra-Dcc Lib initiieren
    _pinMode( ackPin, OUTPUT );
    _digitalWrite( ackPin, LOW );

    Dcc.pin( digitalPinToInterrupt(dccPin), dccPin, 1); 
    if ( progMode == NORMALMODE || progMode == INIMODE ) {
        // keine POM-Programmierung
        Dcc.init( MAN_ID_DIY, version, FLAGS_DCC_ACCESSORY_DECODER | FLAGS_OUTPUT_ADDRESS_MODE, (uint8_t)((uint16_t) 0) );
        CLR_PROGLED;
    } else {
        // POM Programmierung aktiv
        Dcc.init( MAN_ID_DIY, version, FLAGS_DCC_ACCESSORY_DECODER | FLAGS_OUTPUT_ADDRESS_MODE, (uint8_t)(cvPomLow) );
        SET_PROGLED;
    }

}

uint8_t ifc_getCV( uint16_t address ) {
    return Dcc.getCV ( address );
}

void ifc_setCV( uint16_t address, uint8_t value ) {
    Dcc.setCV ( address, value );
}

uint16_t ifc_getAddr(){
    #if defined(NMRADCC_VERSION) && NMRADCC_VERSION >= 200
    return Dcc.getAddr(); 
    #else 
    // wegen Fehler in älteren Versionen der Lib bei Adressen>255, die Adresse selbst berechenen
    uint8_t CV29Value = Dcc.getCV( CV_29_CONFIG ) ; 

    if( CV29Value & CV29_OUTPUT_ADDRESS_MODE ) 
      return ( Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_MSB ) << 8 ) | Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB );
    else
      return ( ( Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_MSB ) & 0b00000111) << 6 ) | ( Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_LSB ) & 0b00111111) ;
    #endif
}

void ifc_process() {
    Dcc.process();
}
// --- Callback-routinen
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
    // mehrere gleiche Telegramme nicht weiterleiten ( vereinfacht debugging)
    static uint16_t lastAddr;
    static uint8_t lastDir;
    static uint8_t lastOp;
    if ( lastAddr!=Addr||lastDir!=Direction||lastOp!=OutputPower ) {
        DB_PRINT("Outout: OAddr=%d Dir=%d, OPow=%d", Addr, Direction, OutputPower );
        //ifc_notifyDccAccState( Addr, BoardAddr, OutputAddr, State );
        ifc_notifyDccAccState( Addr, Direction, OutputPower );
        lastAddr=Addr; lastDir=Direction; lastOp=OutputPower;
    }
}

void notifyCVAck ( ) {
    ifc_notifyCVAck ( );
}

void notifyCVChange( uint16_t CvAddr, uint8_t Value ) {
    ifc_notifyCVChange( CvAddr, Value );
}

void notifyCVResetFactoryDefault(void) {
    ifc_notifyCVResetFactoryDefault();
}

void notifyDccReset( uint8_t hardReset ) {
    ifc_notifyDccReset( hardReset );
}

#endif
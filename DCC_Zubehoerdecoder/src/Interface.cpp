/* Interface des Zubehördecoders
** Alle Interface-abhängigen ( Loconet oder DCC ) Programmkomponenten werden hier
** zusammengefasst, und neutrale Aufrufe für die Funktionalitäten im Sketch zur Verfügung gestellt.
*/
#include "../Interface.h"

#ifdef LOCONET
// --------------- Loconet-Interface -------------------------------------------------
#include <LocoNet.h>
LocoNetSystemVariableClass sv;
lnMsg       *LnPacket;
SV_STATUS   svStatus = SV_OK;
boolean     deferredProcessingNeeded = false;


void ifc_init( uint8_t version, uint8_t progMode ) {
    LocoNet.init(TX_PIN); 
    sv.init(13, 4, 1, 1);
    sv.writeSVStorage(SV_ADDR_NODE_ID_H, 1 );
    sv.writeSVStorage(SV_ADDR_NODE_ID_L, 0);

    sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 0x56);
    sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_L, 0x78);

}

void ifc_process() {
    
}

void notifySVChanged(uint16_t Offset){
    ifc_notifyCVChange( Offset, sv.readSVStorage(Offset) );
}

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
    Dcc.pin( digitalPinToInterrupt(dccPin), dccPin, 1); 
    if ( progMode == NORMALMODE || progMode == INIMODE ) {
        // keine POM-Programmierung
        Dcc.init( MAN_ID_DIY, version, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)((uint16_t) 0) );
        CLR_PROGLED;
    } else {
        // POM Programmierung aktiv
        Dcc.init( MAN_ID_DIY, version, FLAGS_DCC_ACCESSORY_DECODER, (uint8_t)(cvPomLow) );
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
    return Dcc.getAddr();
}

void ifc_process() {
    Dcc.process();
}
// --- Callback-routinen
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State ) {
    ifc_notifyDccAccState( Addr, BoardAddr, OutputAddr, State );
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
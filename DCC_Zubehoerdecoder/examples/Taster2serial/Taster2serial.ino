// Tastendrücke in serielle Kommandos für DIY-Decoder umwandeln
#define MAX32BUTTONS    // es können bis zu 32 Taster definiert wrden
#include <MobaTools.h>
#define SERIAL Serial   // verwendete Schnittstelle
// define pin numbers and assigned adresses/states
const byte buttonPin [] = { 2, 3, 4, 5 };
const byte adresses [] =  {17,17,18,18 };   // DCC-adresses ( Output  adressing )
const byte states []   =  { 0, 1, 0, 1 };   // state ( 0 or 1 );
const byte buttonCnt = sizeof(buttonPin);
MoToButtons Buttons( buttonPin, buttonCnt, 30, 500, 400 );
char cmdBuf[20];

void setup()
{
  SERIAL.begin(115200);
}

void loop() {
  //--------------------------------------------------------
  // read and process buttons
  Buttons.processButtons();
  // 
  //--------------------------------------------------------
  for ( byte i=0; i<buttonCnt; i++ ) {
    if ( Buttons.pressed(i) ) {
      sprintf(cmdBuf, "ac %d %d 1", adresses[i], states[i] );
      SERIAL.println(cmdBuf);
    }
    if ( Buttons.released(i) ) {
      sprintf(cmdBuf, "ac %d %d 0", adresses[i], states[i] );
      SERIAL.println(cmdBuf);
    }
  }
}

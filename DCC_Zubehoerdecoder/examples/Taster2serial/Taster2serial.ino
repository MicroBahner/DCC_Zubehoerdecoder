// Tastendrücke in serielle KOmmandos für DIY-Decoder umwandeln
#define MAX8BUTTONS
#include <MobaTools.h>
// define pin numbers
const byte buttonPin [] = { 2,3,4,5 };
const byte buttonCnt = sizeof(buttonPin);
MoToButtons Buttons( buttonPin, buttonCnt, 30, 500, 400 );

void setup()
{
  Serial.begin(115200);
}

void loop() {
  //--------------------------------------------------------
  // read and process buttons
  Buttons.processButtons();
  // 
  //--------------------------------------------------------
  if ( Buttons.pressed(0) ) Serial.println("ac 17 0 0");
  if ( Buttons.pressed(1) ) Serial.println("ac 17 1 0");
  if ( Buttons.pressed(2) ) Serial.println("ac 18 0 0");  
  if ( Buttons.pressed(3) ) Serial.println("ac 18 1 0");
}

# DCC_Zubehoerdecoder
### A universal Dcc Accessory-Decoder for Arduino based on NmraDcc and MoBaTools Lib. 

It is able to control servos, double coil motors and leds. Most characteristics are CV configurable. 

Overview of the adjustable functionality:
- Servo with relay for turnout polarization
- logical coupling of 2 servos for 3-aspect form signals
- 1 servo via 2 addresses to control 4 positions
- Impulse function for servos (automatic return to initial position, e.g. for decoupler)
- Double-coil drives
- static outputs
- flashing outputs
- Light signal functions. The signal aspects are switched smoothly, with a short dark phase between the signal aspects.

The decoder may alternatively be used with a LocoNet Interface ( Only Arduino Leonardo/Micro or Mega )

### Ein universeller DCC/LocoNet-Zubehördecoder, basierend auf der NmraDCC/LocoNet und der MobaTools Lib.

Es lassen sich Servo's, Doppelspulenantrieb und Led's ansteuern.

Übersicht über die einstellbare Funktionalität:
- Servo mit Umschaltrelais zur Weichenpolarisierung
- logische Kopplung von 2 Servos für 3-begriffige Formsignale
- 1 Servo über 2 Adressen um 4 Positionen anzusteuern
- Impulsfunktion für Servos ( automatisches Rückkehren in Ausgangslage, z.B. für Entkuppler )
- Doppelspulenantriebe
- statische Ausgänge
- blinkende Ausgänge
- Lichtsignalfunktionen. Die Signalbilder werden weich umgeschaltet, mit einer kurzen Dunkelphase zwischen den Signalbildern.

Mit LocoNet Interface läuft der Decoder nur auf Arduino Leonardo, Micro oder Mega.
Mit DCC-Interface auch auf UNO/Nano und STM32F1 Prozessoren  ( mit dem Core von Roger Clark: [Arduino_STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32) )


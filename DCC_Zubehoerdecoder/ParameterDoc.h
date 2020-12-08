/* Universeller DCC - Zubehördecoder
 * 3 Ausgänge / Zubehöradresse
 * Einstellbare Funktionalität:
 *  - Servo mit Umschaltrelais zur Weichenpolarisierung
 *  - Doppelspulenantriebe
 *  - statische/blinkende Ausgänge 
 *  - Lichtsignale 
 *  - Einstellen der Servoendlagen per Drehencoder. 
 *    Der Drehencoder bezieht sich immer auf die zuletzt gestellte Adresse und Position.
 *  - Die Betriebsmodi und das Startverhalten werden über die Analogeingänge A4/A5 ( betrModeP und 
 *    resModeP, parametrierbar) eingestellt. Dazu müssen dort Pullups eingebaut werden. 
 *    Je nachdem wieweit die Spannung  heruntergezogen wird werden die Modi eingestellt:
 *      betrModeP:
 *        5V (offen) normaler Betriebsmodus, kein PoM
 *        3,3V (Spannungsteiler 1:2) PoM immer aktiv, Adresse immer aus defaults
 *        1,6V (Spannungsteiler 2:1) IniMode: CV's werden immer auf init-Werte aus .h-Datei gesetzt
 *        0V Programmiermodus / PoM ( 1. Empfamgenes Telegramm bestimmt Adresse )
 *      resModeP:
 *        wird resModeP auf 0 gezogen , wird der aktuell vom Drehencoder beeinflusste Servo in die  
 *           Mittellage gebracht. Sobald der Encoder wieder bewegt wird, bewegt sich das Servo wieder
 *           zur vorhergehenden Position.
 *           Ist resModeP beim Programmstart auf 0, werden alle CV's auf die Defaults zurückgesetzt
 *                  
 * Eigenschaften:
 * Mehrere (aufeinanderfolgende) Zubehöradressen ansteuerbar. Die mögliche Zahl von Adressen hängt 
 * von den verfügbaren Digitalausgängen ab.
 * Es können maximal 16 Servos angesteuert werden
 * 
 * 1. Adresse per Programmierung einstellbar
 * 
 *  Das Verhalten der konfigurierten Funktionen wird über CV-Programmierung festgelegt: 
 *  Bei Servoausgängen die Endlagen und die Geschwindigkeit
 *  bei Doppelspulenantrieben die Einschaltzeit der Spulen.
 *  bei blinkenden Ausgängen das Blinkverhalten ( in V3.0 noch nicht realisiert )
 *  
 *  Aufteilung der CV's:
 *  CV      Bedeutung    
 *  47      Kennung für Erstinitiierung, allgemeine Optionen die für den gesamten Decoder gelten
 *  48/49   Pom-Adresse
 *  50-54   Parameter für 1. Weichenadresse
 *  55-59   Parameter für 2. Weichenadresse
 *  ...
 *  Bedeutung der CV's bei den verschiedenen Funktione (CV-Nummern für 1. Weichenadresse)
 *  FSERVO Servo:
 *  CV50    Bit0 = 1: (SAUTOOFF) AutoOff der Servoimpulse bei Stillstand des Servo
 *          Bit1 = 1: (SDIRECT) 'Direct-Mode' auch während der Servobewegung wird auf einen erneuten
 *                    Stellbefehl reagiert, und gegebenenfalls sofort die Drehrichtung geändert
 *          Bit2 = 1: (SAUTOBACK) Automatiche Rückkehren in die Ausgangslage nach Zeit in CV54
 *          Bit3 = 1: (NOPOSCHK) kein Überprüfung auf Servoposition bei Empfang eines DCC-Befehls
 *                    bei AUTOOFF und gleicher Position werden wieder Impulse ausgegeben
 *  CV51    Position des Servo für Weichenstellung '0' ( in Grad, 0...180 )
 *  CV52    Position des Servo für Weichenstellung '1' ( in Grad, 0...180 )
 *  CV53    Geschwindigkeit des Servo
 *  CV54    Wenn CV50 Bit4= 1: Zeit in 0,1Sek. Einheiten bis zum automatisch zurückbewegen.
 *          In diesem Fall startet das Servo beim Einschalten grundsätzlich in der Grundstellung
 *  
 *  FSERVO0 verbundene Servos, muss unittelbar hinter FSERVO stehen ( Folgeadresse )
 *         Ist die Pinnr identisch zum Eintrag unter FSERVO, so wird nur 1 Servo eingerichtet, 
 *         dass auf 4 Positionen gestellt werden kann. Die Einträge von CV50 und CV53/54 sind
 *         dann im FSERVO0 Eintag belanglos.
 *         Enthält FSERVO0 eine unterschiedliche Pinnr für das Servo, so werden zwei Servos
 *         eingerichtet. Das Modusbyte von FSERVO gilt dann fur beide. Die Lage der Servos kann
 *         den DCC-Kommandos frei zugeordnet werden. Damit können z.B. 3-begriffige Formsignale 
 *         gesteuert werdem.
 *  CV50   Bitmuster, das die Lage der Servos für die 4 möglichen Befehle angibt
 *         Bit=0 Ruhelage ( Position CV51 ) , Bit=1 Arbeitslage ( Position CV52 )
 *         Das niederwertige Bit steuert jeweils das Servo FSERVO,
 *         das höherwertige  Bit steuert das Servo FSERVO0
 *         Bit76543210
 *                  ++-- OFF 1.Adresse
 *                ++---- ON  1.Adresse
 *              ++-------OFF 2.Adresse
 *            ++-------- ON  2.Adresse
 *  CV51 .. 54 wie bei FSERVO        
 *            
 *  F2SERVO Servo: ( 2 Servos auf einer Adresse )
 *  CV50..CV54  wie beim ersten Servo (FSERVO)
 *  CV55..CV59  Paramtersatz des 2. Servo ( Aufteilung wie beim 1. Servo )
 *  !!!Wird nur bei einerm Servo das SAUTOBACK-Flag gesetzt, so muss dies das 1. Servo sein.
 *            
 *  FCOIL Doppelspulenantrieb: ( derzeit nur mit automatischer Abschaltung )
 *  CV50    Bit0 = 1: (CAUTOOFF) Spulenausgang nur automatisch abschalten
 *               = 0: Spulenausgang auch über DCC-Befehl abschalten
 *          Bit3 = 1: (NOPOSCHK) kein Überprüfung auf Weichenposition. Gegebenenfalls wird auch an gleichen
 *                    Anschluss wiederholt ein Puls ausgegeben
 *  CV51    Einschaltdauer der Spule  ( in 10ms Einheiten ) 
 *          0= keine automatische Abschaltung, Bit0 im Modebyte muss 0 sein
 *  CV52    minimale Ausschaltdauer der Spule ( in 10ms Einheiten )
 *  CV53    -
 *  CV54    aktuelle Weichenstellung ( nicht manuell verändern! )
 *  
 *  FSTATIC statischer/Blinkender Ausgang 
 *  CV50    Bit0 = 1: (BLKMODE) Blinken,  0: statisch
 *          Bit1 = 1: (BLKSTRT) Beim Blinken starten erst beide Leds dann Wechselblinken
 *          Bit2 = 1: (BLKSOFT) mit weichem Auf/Abblenden 
 *          Bit4..7:  Risetime ( in 50ms Einheiten, 0=default von 500 )
 *  CV51    Einschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV52    Ausschaltzeit des Blinkens ( 10ms Einheiten )
 *  CV53    1. Einschaltzeit beim Start des Blinkens
 *  CV54    aktueller Zusatnd ( nicht manuell verändern! )
 *  
 *  FSIGNAL2 Lichtsignalfunktion mit 1..3 Weichenadressen 
 *          bei den Folgeadressen ist als Typ FSIGNAL0 einzutragen
 *          Lichtsignale starten beim Einschalten immer im Zustand 0 (Bitmuster CV51)
 *  CV50    Signalmodus: Bit7=1 : invertiert die Softled-Ausgänge (HIGH=OFF) (MobaTools ab V0.9)
 *          Bit 2..0: Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *          und welche Ausgänge weich überblenden (Bit=0)
 *  CV51    Bitmuster der Ausgänge für Befehl 1.Adresse 0 (rot)
 *  CV52    Bitmuster der Ausgänge für Befehl 1.Adresse 1 (grün)
 *  CV53    Index des Vorsignals am gleichen Mast ( 1 …. )
 *  CV54    Bitmuster der Zustände, bei denen das Vorsignal dunkel ist:
 *              Bit 0: Befehl 1.Adresse 0 (rot)
 *              Bit 1: Befehl 1.Adresse 1 (grün)
 *              Bit 2: Befehl 2.Adresse 0 (rot)
 *              u.s.w.
 *  CV55    Bitmuster der blinkenden Leds für Befehl 1.Adresse 0 (rot)
 *  CV56    Bitmuster der blinkenden Leds für Befehl 1.Adresse 1 (grün)
 *  CV57-58 reserved
 *  CV59    Blinktakt ( 10ms-Einheiten ) 
 
 *  FSIGNAL0 1. Folgeadresse (optional)
 *  CV60    Bit 2.. 0 Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *          und Welche Ausgänge weich überblenden (Bit=0)
 *  CV61    Bitmuster der Ausgänge für Befehl 2.Adresse 0 (rot)
 *  CV62    Bitmuster der Ausgänge für Befehl 2.Adresse 1 (grün)
 *  CV63    reserved
 *  CV64    reserved
 *  CV65    Bitmuster der blinkenden Leds für Befehl 2.Adresse 0 (rot)
 *  CV66    Bitmuster der blinkenden Leds für Befehl 2.Adresse 1 (grün)
 *  CV67-69 reserved
 
 *  FSIGNAL0 2. Folgeadresse (optional)
 *  CV70    Bit 2.. 0 Bitmuster hard/soft gibt an, welche Ausgänge 'hart' umschalten (Bit=1)
 *          und Welche Ausgänge weich überblenden (Bit=0)
 *  CV71    Bitmuster der Ausgänge für Befehl 3.Adresse 0 (rot)
 *  CV72    Bitmuster der Ausgänge für Befehl 3.Adresse 1 (grün)
 *  CV73    reserved
 *  CV74    reserved
 *  CV75    Bitmuster der blinkenden Leds für Befehl 3.Adresse 0 (rot)
 *  CV76    Bitmuster der blinkenden Leds für Befehl 3.Adresse 1 (grün)
 *  CV77-79 reserved
 *  
 *  FVORSIG Vorsignalfunktion
 *          weitgehend wie FSIGNAL2 ausser:
 *  CV53    low Byte der Adresse des angekündigten Hauptsignals
 *  CV54    high Byte der Adrsse des angekündigten Hauptsignals
  */

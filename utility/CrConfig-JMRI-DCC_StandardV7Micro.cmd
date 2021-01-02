@echo off
rem Modell-Name des Decoders in JMRI ( Buchstaben,Zahlen und _ ):
set model=DIY_StandardV7MicroDcc
rem Interface: DCC oder LocoNet. LcoNet ist standard
set interface=DCC
rem Konfiguration des Decoders (Funktionen):
rem Bitte keine Zeilen lösche, nur den Wert hinter dem = anpassen
rem Hinter den Funktionsnamen dürfen keine Leerzeichen stehen und
rem es dürfen keine Lücken entstehen. Nicht verwendete Funktionen bleiben am Ende frei
set INITYP1=FSERVO
set INITYP2=FSERVO0
set INITYP3=FSIGNAL2
set INITYP4=FSIGNAL0
set INITYP5=FVORSIG
set INITYP6=FCOIL
set INITYP7=F2SERVO
set INITYP8=
set INITYP9=
set INITYP10=
set INITYP11=
set INITYP12=
rem ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
rem ------------ Ende des Benutzeranpassbaren Bereiches ---------------------------------

echo #define _MODEL %model%> %model%.txt
echo #define _INTERFACE %interface%>> %model%.txt
echo #define INITYP1 %INITYP1%>> %model%.txt
echo #define INITYP2 %INITYP2%>>  %model%.txt
echo #define INITYP3 %INITYP3%>>  %model%.txt
echo #define INITYP4 %INITYP4%>>  %model%.txt
echo #define INITYP5 %INITYP5%>>  %model%.txt
echo #define INITYP6 %INITYP6%>>  %model%.txt
echo #define INITYP7 %INITYP7%>>  %model%.txt
echo #define INITYP8 %INITYP8%>>  %model%.txt
echo #define INITYP9 %INITYP9%>>  %model%.txt
echo #define INITYP10 %INITYP10%>>  %model%.txt
echo #define INITYP11 %INITYP10%>>  %model%.txt
echo #define INITYP12 %INITYP10%>>  %model%.txt
gpp -s "\"" +n  --include %model%.txt  -I .\src  -o Public_Domain_%interface%_%model%.xml src\Public_Domain_GPP-Generic.gpp
rem pause
del %model%.txt
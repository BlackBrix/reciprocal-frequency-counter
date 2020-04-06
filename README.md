# reciprocal-frequency-counter  
reciprocal frequency counter with Arduino Uno R3 (0.016Hz - 250kHz* at 16MHz processor clock)   
*(optional up to 7MHz see below)  
  
originally from Michael Nowak:    
http://www.mino-elektronik.de
http://www.mino-elektronik.de/fmeter/fm_software.htm#bsp7
  
## German description:  

Frequenz-Periode-Drehzahl, angepasste Version für Arduino UNO R3  
  
Basierend auf dem hier verwendeten Programm ist das Programm Fmeter_UNOR3.ino für die direkte Verwendung auf dem genannten Board angepasst. Der Vorteiler und die LED-Anzeige entfallen. Die Datenausgabe erfolgt über die RS232-Schnittstelle, die direkt von einen PC über einen USB-Port zur Verfügung steht.  
  
Das Eingangssignal wird an IO7 (PD7/AIN1) erwartet und kann im Bereich 0,016Hz - ca. 250kHz liegen.  
Gemessen werden Frequenz, Periode oder Drehzahl in Abhängigkeit des Signals an A0 (ADC0).  
  
'1' oder offFrequenz-Periode-Drehzahl, angepasste Version für Arduino UNO R3

Basierend auf dem hier verwendeten Programm ist das Programm Fmeter_UNOR3.ino für die direkte Verwendung auf dem genannten Board angepasst. Der Vorteiler und die LED-Anzeige entfallen. Die Datenausgabe erfolgt über die RS232-Schnittstelle, die direkt von einen PC über einen USB-Port zur Verfügung steht.

Das Eingangssignal wird an IO7 (PD7/AIN1) erwartet und kann im Bereich 0,016Hz - ca. 250kHz liegen.
Gemessen werden Frequenz, Periode oder Drehzahl in Abhängigkeit des Signals an A0 (ADC0).

'1' oder offener Eingang: Frequenz
'0' oder Brücke nach GND: Periode
Vcc/2 oder 10-47k nach GND: Drehzahl

Die Ergebnisse werden 6-stellig mit entsprechender Angabe der Dimension per RS232 (IO1 bzw. TXD/PD1) ausgegeben oder per USB-Anschluss, wie es der Arduino UNO R3 mit seinem 'Serial Monitor' bietet.

automatischer Abgleich mit 1Hz-Referenzsignal / GPS per Eingangs A1 (ADC1):
Für diesen Abgleich wird das 1Hz Signal an den Signaleingang gelegt und über Tools/Serial Monitor die Datenausgabe verfolgt. Die angezeigte Frequenz muss um 1Hz mit +/-1000ppm liegen, was 0,999 Hz - 1.001 Hz entspricht. Der Eingang A1 wird nun gegen 0V/GND geschaltet und damit der automatische Abgleich gestartet. Die Meldung "Abgleich fertig" zeigt an, dass der Korrkturwert berechnet und abgespeichert wurde. Anschliessend werden die Ergebnisse mit "1.00000 Hz" angezeigt.

Der Aduino UNO R3 ist mit einem kermischen Resonator 16MHz bestückt, der keine sonderlich genaue oder stabile Taktfrequenz bietet. Die Frequenz ändert sich mit schwankender Temperatur sehr deutlich. Um dies zu verbessern, kann man den keram. Resonator entfernen und den 16MHz Takt des ATmega16U2 verwenden, der mit einem 16 MHz Quarz erzeugt wird. Dazu werden die beiden Anschlüsse XTAL1 auf der Platinenunterseite mit einer kurzen Leitung verbunden: Pin1 des ATmega16U2 an Pin9 des ATmega328. Ein Bild macht dies deutlich.
Diese Lösung ist technisch nicht ganz perfekt, funktioniert aber und liefert deutlich bessere Stabilität. Ein separater Quarz nebst 2 x 22pF Kondensatoren wäre eine bessere Lösung.

Wenn alle sechs Stellen über die Temperatur bzw. Zeit stabil bleiben sollen, ist ein TCXO als Taktgeber notwendig. Das ist die aufwendigste aber auch beste Lösung.

### 2015-09-03: Vorteiler bis ca. 7 MHz
Eine weitere Programmversion für den Arduino verwendet den Timer0 als Vorteiler 1/100, so dass Frequenzen bis ca. 7,4 MHz gemessen werden können. Der Eingang des Vorteilers liegt an IO4 (PD4/T0) und der Ausgang an IO6 (PD6/OC0A) der per Drahtbrücke direkt auf IO7 geschaltet wird: Fmeter_UNOR3_mod.ino. Ferner ist der Messablauf geändert, so dass die Auswertung (nicht aber die Messung selbst!) nicht mehr synchron zum Eingangssignal passiert, sondern im vorgegebenen Zeitraster. Im Programm sind dies 100 ms, die gewartet werden, bis die bis dahin eingetroffenen Impulse ausgewertet werden. Da minimal ein Eingangsimpuls benötigt wird, wird dieses Zeitraster erst ab Frequenzen > 10 Hz wirksam; bei niedrigeren Frequenzen muss entsprechend länger (eine ganze Periode) gewartet werden. Hierdurch werden bei genügend hoher Eingangsfrequenz im Mittelwert immer 10 Messungen/s ausgeführt. Weiterhin wird bei fehlendem Eingangssignal der Wert "0.00000 <dim>" ausgegeben. Die Zeit wird durch den Wert von TIMOUT vorgegeben. Ein Wert von 1000 entspricht einer Sekunde und kann zum Beispiel bei der Drehzahlmessung den Stillstand (<= 60 U/min) anzeigen, der dann jede Sekunde erneut ausgegeben wird.

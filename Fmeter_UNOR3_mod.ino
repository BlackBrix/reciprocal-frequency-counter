/*
reziproker Frequenzzaehler mit 6-stell. Datenausgabe ueber RS232
angepasst fuer ARDUINO UNO R3.
Funktionsbereich etwa 0,016 Hz - 250 kHz bei 16 MHz Prozessortakt 
Das Messsignal wird an IO7 (PortD.7/AIN1) erwartet.

Gemessen werden Frequenz, Periode oder Drehzahl in Abhaengigkeit des Signals an A0 (ADC0).
'1' oder offener Eingang:   Frequenz
'0' oder Bruecke nach GND:   Periode
Vcc/2 oder 10-47k nach GND: Drehzahl

Ausgabe des Messwertes ueber RS232 mit DEF_BAUDRATE mit ensprechender Dimension.

Das interne Timing basiert auf einem ms-Raster. Zusaetzlich wird an IO9 (PB1/OC1A) ein 1kHz 
Signal ausgegeben, welches als Referenz oder zum Abgleich verwendet werden kann.
Voraussetzung hierfuer ist die Verwendung einer Quarzfrequenz, die /2000 ohne Rest teilbar
ist. Zum Beispiel 20 MHz oder auch 18,432 MHz.

automatischer Abgleich mit 1 Hz-Referenzsignal / GPS:
Fuer diesen Abgleich wird das 1 Hz Signal an den Signaleingang gelegt und ueber "Tools/Serial Monitor"
die Datenausgabe verfolgt. Die angezeigte Frequenz muss um 1 Hz mit +/-1000 ppm liegen, was
0,999 Hz - 1.001 Hz entspricht. Der Eingang A1 wird nun gegen 0V/GND geschaltet und damit der 
automatische Abgleich gestartet. Die Meldung "Abgleich fertig" zeigt an, dass der Korrkturwert
berechnet und abgespeichert wurde. Anschliessend werden die Ergebnisse mit "1.00000 Hz" angezeigt.
Da der UNO R3 keine besonders stabile Quarzfrequenz bietet, wird sich dieser Wert mit schwankender
Temperatur mehr oder weniger deutlich veraendern. Ein TCXO ist notwendig, wenn alle sechs Stellen
ueber die Temperatur bzw. Zeit stabil bleiben sollen.

http://www.mino-elektronik.de
Alle Angaben wie immer ohne Gewaehr !
2013-09-16

2015-09-03:
modifizierte Version mit Timer0 als Vorteiler 1/100 fuer hoehere Eingangsfrequenzen <= 8 MHz und geaenderter Messzeit.
Eingangssignal an IO4 (PortD.4); Ausgangssignal an IO6 (PortD.6) an IO7 (PortD.7) anschliessen (Drahtbruecke).
Die max. Eingangsfrequenz liegt bei ca. 7,4 MHz.
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define BIT(x)  (1<<x)

#define F_CPU           16000000L       // 16 MHz ggf. anpassen
#define DEF_BAUDRATE    19200

#define MS_TEILER       (F_CPU/1000)    // Intervall fuer 1ms
#define VORTEILER       100             // ueber Timer0

#define STELLEN         6               // angezeigte Stellen
#define MESSZEIT        100             // 10 Messungen/Sek. 
// #define TIMEOUT         65000            // max. 65 s warten
#define TIMEOUT         1000            // max. 1 s warten
#define EE_ADR_OFFSET   2               // Platz fuer Korrekturwert
#define KAL_TASTE       1               // PORTC-1

enum status {MESSEN=0, AUSLESEN, AUSWERTEN};    // die Phasen der Messung
enum anzeige_typ {FREQUENZ, PERIODE, DREHZAHL}; // zur Auswahl

float frequenz;                         // float-Ergebnis
float f_clock;                          // ggf. mit Korrekturwert

volatile uint8_t messwert_vorhanden,    // nur gueltige Messungen anzeigen
        mess_status;                    // Ablaufsteuerung

volatile uint16_t mess_dauer,           // minimale Wartezeit
        ueberlauf;

volatile uint32_t end_ereignis,         // Impulse am Ende der Messung
        start_zeit,                     // rel. Zeit: Beginn der Messung
        end_zeit,                       // Zeitpunkt der Auswertung
        mess_zeit,                      // genaue Zeit der Messung
        mess_ereignisse;                // Impulse der Messung

volatile uint32_t count, zeitpunkt;     // Impulse per Interrupt                


const uint8_t f_dim[][4]={"GHz", "MHz","kHz","Hz ","mHz",           // Dimension fuer RS232-Ausgabe
                          "sek","ms ","us ","ns ","ps "};
const uint8_t u_dim[][6]={"Grpm ","Mrpm ","krpm ","rpm ","mrpm "};

uint8_t lese_adc(uint8_t kanal)             // nur 8 Bit verwenden
{
  ADMUX =  BIT(REFS0) | BIT(ADLAR) | kanal; // Vcc als Vref, linksbuendig
  ADCSRA |= BIT(ADSC);                      // starten
  while(ADCSRA & BIT(ADSC));                // warten
  return(ADCH);                             // fertig  
}

uint8_t anzeige_wert(void)                  // Spannung an ADC0 als Messmodus bewerten
{
uint8_t temp = lese_adc(ADCH0);              
  if(temp >= 230) return(FREQUENZ);         // bei offenem Eingang FREQUENZ
  if(temp <= 20)  return(PERIODE);          // bei Kurzschluss PERIODE
  return(DREHZAHL);                         // ansonsten DREHZAHL
}

void init_serio0(void)
{
uint8_t temp;
  UCSR0B = 0x0;                             // alles abschalten
  UCSR0C = BIT(UCSZ01) | BIT(UCSZ00);       // default wie nach reset 8 Bit, async, no par.
  UBRR0=((F_CPU/16L/(DEF_BAUDRATE/10L)+5L)/10L) - 1;    // passend gerundet
  UCSR0B |= BIT(RXEN0) | BIT(TXEN0);        // RX + TX freigeben
  temp = UDR0;
}

void ser_putchar(uint8_t z)
{
  while(!(UCSR0A & BIT(UDRE0)));
  UDR0 = z;
}

void ser_string(uint8_t *s)
{
  while(*s) ser_putchar(*s++);
  ser_putchar(13);
  ser_putchar(10);
}

void init_timer_adc(void)
{
  OCR0A = VORTEILER/2-1;
  TCCR0A = BIT(COM0A0) | BIT(WGM01); // toggle OCR0A-Ausgang
  TCCR0B = 0x07;                    // CTC-Modus T0 zaehlt Impulse vom Eingang T0
  
  OCR1A = MS_TEILER;                // ms-Intervall fuer Timing
  OCR1B = MS_TEILER/2;              // 0,5 ms
  TCCR1A |= BIT(COM1A0);            // togglen mit OCR1B Intervall -> 1 kHz Ausgang
  TCCR1B = 0x01;                    // Timer1 ohne Vorteiler
  TIMSK1 = BIT(ICIE1) |             // ICP1 Int-enable Frequenz vom Vorteiler
           BIT(TOIE1) |             // OVFL Int-enable 
           BIT(OCIE1A) |            // COMP1A Int-enable fuer 1 Khz Signal
           BIT(OCIE1B);             // COMP1B Int-enable fuer ms timing
  ADCSRA = BIT(ADEN) + 0x07;        // einschalten mit Teiler /128

}

// Routine zur Wandlung und Ausgabe eines Messwertes

void zeige_x(double x, int8_t typ, int8_t mit_rs232)
{
int8_t i,j,k,dimension;
  k=-6,dimension=1;                         // default Frequenzanzeige
  if(x!=0) {
    if(typ == DREHZAHL) {
      x *= 60.0;
    }
    else if(typ == PERIODE) {
      x=1/x;
      dimension=3;
    }
    if(x >= 1e9) {x*=0.001;dimension--;}
    while(x<1e6) {x*=1000;dimension++;}
// x liegt im Bereich 1e6 - 9.99999e8
    while(x>=10.0) {
      x*=0.1;
      k++;
    }
// x liegt im Bereich 1.0 - 9.99999
    x += 5e-6;                              // aufrunden
    if(x >= 10.0) {                         // und ggf. Bereich korrigieren
      x *= 0.1;
      k++;
      if(k > 2) {
        dimension--;
        k=0;
      }
    }
  } else { k=0;dimension=3;}                // fuer x = 0.0

  for(i = 0; i < STELLEN; i++) {
    j = x;
    ser_putchar(j+'0');
    if(i == k) {
      ser_putchar('.');
    }
    x-=j;
    x*=10.0;
  }

  ser_putchar(' ');
  if(typ == DREHZAHL) 
    ser_string((uint8_t *)u_dim[dimension]);        
  else
    ser_string((uint8_t *)f_dim[dimension]);
}

// an IO9 (PB1/OC1A) ein 1kHz Signal ausgeben
ISR(TIMER1_COMPA_vect)              // wird mit 2000 Hz aufgerufen
{
  OCR1A += MS_TEILER/2;         
}

// 1kHz Interrupt für ms-genaues Timing
ISR(TIMER1_COMPB_vect)              // wird mit 1000 Hz aufgerufen
{
  OCR1B += MS_TEILER;         
  mess_dauer++;                     // Dauer der Messung
  if(mess_status == MESSEN && mess_dauer >= MESSZEIT && count) {
    end_ereignis = count;           // Anzahl der Impulse lesen
    end_zeit = zeitpunkt;
    mess_dauer = count = 0;         // wieder loeschen
    mess_status = AUSWERTEN;        // Daten fertig fuer Auswertung
  }
}

// Routinen zur Erfassung der Eingangsimpulse
// Überläufe von T1
ISR(TIMER1_OVF_vect)            
{
  ueberlauf++;                     // Ueberlaeufe von T1 ergeben obere 16bit der Messzeit
}

// CAPT-Ereignisse an AIN1
ISR(TIMER1_CAPT_vect)              // Eingangsimpulse mit genauem Zeitpunkt erfassen
{
uint16_t zeit_l, zeit_h;  
  count++;
  zeit_l = ICR1;                   // capture-reg lesen: untere 16bit
  zeit_h = ueberlauf;              // dazu die oberen 16bit
  if((TIFR1 & BIT(TOV1)) && (zeit_l < 0x8000))    // evtl. Ueberlauf T1 noch offen?
    zeit_h++;                      // nur, wenn capture-int + overflow-int gleichzeitig !
  zeitpunkt = zeit_h*0x10000 + zeit_l;
}


uint8_t ee_read_byte(uint16_t adr)
{
  while(EECR & BIT(EEPE));        // abwarten
  EEAR = adr;
  EECR |= BIT(EERE);              // lesebit setzen
  return(EEDR);
}

void ee_write_byte(uint8_t data, uint16_t adr)
{
  if(data != ee_read_byte(adr)) {
    cli();
    EEAR = adr;
    EEDR = data;
    EECR  = BIT(EEMPE);          // master schreibbit setzen, EEPMx loeschen
    EECR |= BIT(EEPE);           // schreibbit setzen
    sei();
  }
}

void ee_write_word(int16_t data, uint16_t adr)
{
  ee_write_byte((uint16_t)(data)%256, adr);
  ee_write_byte((uint16_t)(data)/256, adr+1);
}

int16_t ee_read_word(uint16_t adr)
{
int16_t temp;
  temp = ee_read_byte(adr);
  temp += ee_read_byte(adr+1) * 256;
  return(temp);
}



/* Test auf 0V / GND an A1 (PortC.1) und externes Referenzsignal von 1Hz.
Wenn beides vorliegt, wird die Referenzfrequenz 'f_clock' um den 
Korrekturwert 'f_offset' angepasst. 'f_offset' wird im EEPROM gespeichert
und nach einem Reset erneut geladen und mit F_CLOCK verrechnet.
*/

void teste_kalibrierung()
{
static uint8_t temp_alt;                            // zur Ermittelung der aktiven Flanke
uint8_t temp;                                       
int16_t f_offset;                                   // Abweichung zu F_CLOCK
  temp = (PINC ^ BIT(KAL_TASTE)) & BIT(KAL_TASTE);  // Zustand des Tasters lesen
  if((temp ^ temp_alt) & temp) {                    // Taste wurde gedrueckt
    if(frequenz > 0.999 && frequenz < 1.001) {      // Frequenzbereich eingrenzen +/-1000 ppm
      f_offset = mess_zeit - F_CPU;                 // Differenz ist-soll ermitteln
      f_clock = (float) (F_CPU + f_offset);         // korrigierter Takt
      ser_string((uint8_t *)("Abgleich fertig"));
      ee_write_word(f_offset, EE_ADR_OFFSET);       // Korrekturwert speichern
      messwert_vorhanden = 0;                       // naechte Messung verwerfen
    }
  }
  temp_alt = temp;                                  // fuers naechste Mal merken
}


// Hauptprogramm
int main(void)
{
  PORTB = BIT(PORTB0);                      // Pullup ICP
  DDRB =  BIT(PORTB1);                      // 1 kHz signal an OC1A
  PORTD = BIT(PORTD4) | BIT(PORTD7);        // Pullup T0-Eingang, und AIN1
  DDRD =  BIT(PORTD6);                      // OCR0A als Ausgang fuer Vorteiler
  PORTC = BIT(ADCH1) | BIT(ADCH0);          // Pullup ADC0+ADC1
  ACSR  = BIT(ACIC) | BIT(ACBG);            // analog Komparator verwenden
 
  mess_status = MESSEN;                     // zuerst messen
  messwert_vorhanden = 0;                   // 1.Ergebnis unterdruecken, da es falsch ist
  f_clock = (float) (F_CPU + ee_read_word(EE_ADR_OFFSET));    // eff. Referenzfrequenz

  init_timer_adc();
  init_serio0();
  sei();

  for(;;) {
    if(mess_dauer > TIMEOUT) {              // Messung abbrechen
      cli();
      count = mess_dauer = 0;
      messwert_vorhanden = 0;               // naechstes Ergebnis verwerfen
      sei();
      zeige_x(0.0,anzeige_wert(),0);        // 0.0 ausgeben
      mess_status = MESSEN;                 // neue Messung zulassen
    }

    if(mess_status==AUSWERTEN) {
      mess_zeit = end_zeit - start_zeit;    // Zeit-Differenz bilden
      start_zeit = end_zeit;
      mess_ereignisse = end_ereignis;       // direkter Wert
      mess_status = MESSEN;                 // neue Messung zulassen
      if(messwert_vorhanden) {              // und gueltigen Wert anzeigen
        frequenz = ((float)mess_ereignisse * f_clock) / mess_zeit;  // Frequenz berechnen
        zeige_x(frequenz * VORTEILER,anzeige_wert(),0); // als Frequenz, Periode oder Drehzahl anzeigen/ausgeben
        teste_kalibrierung();               // nach gueltiger Messung ggf. neu abgleichen
      }
      else messwert_vorhanden = 1;          // sperre wieder aufheben
    }
  }
}


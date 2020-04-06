/*
reziproker Frequenzzähler mit 6-stell. Datenausgabe über RS232
angepaßt für ARDUINO UNO R3.
Funktionsbereich etwa 0,016Hz - 250kHz bei 16MHz Prozessortakt 
Das Meßsignal wird an IO7 (PortD.7/AIN1) erwartet.

Gemessen werden Frequenz, Periode oder Drehzahl in Abhängigkeit des Signals an A0 (ADC0).
'1' oder offener Eingang:   Frequenz
'0' oder Brücke nach GND:   Periode
Vcc/2 oder 10-47k nach GND: Drehzahl

Ausgabe des Messwertes über RS232 mit DEF_BAUDRATE mit ensprechender Dimension.

Das interne Timing basiert auf einem ms-Raster. Zusätzlich wird an IO9 (PB1/OC1A) ein 1kHz 
Signal ausgegeben, welches als Referenz oder zum Abgleich verwendet werden kann.
Voraussetzung hierfür ist die Verwendung einer Quarzfrequenz, die /2000 ohne Rest teilbar
ist. Zum Beispiel 20MHz oder auch 18,432MHz.

automatischer Abgleich mit 1Hz-Referenzsignal / GPS:
Für diesen Abgleich wird das 1Hz Signal an den Signaleingang gelegt und über "Tools/Serial Monitor"
die Datenausgabe verfolgt. Die angezeigte Frequenz muß um 1Hz mit +/-1000ppm liegen, was
0,999 Hz - 1.001 Hz entspricht. Der Eingang A1 wird nun gegen 0V/GND geschaltet und damit der 
automatische Abgleich gestartet. Die Meldung "Abgleich fertig" zeigt an, dass der Korrkturwert
berechnet und abgespeichert wurde. AnschlieÃŸend werden die Ergebnisse mit "1.00000 Hz" angezeigt.
Da der UNO R3 keine besonders stabile Quarzfrequenz bietet, wird sich dieser Wert mit schwankender
Temperatur mehr oder weniger deutlich verändern. Ein TCXO ist notwendig, wenn alle sechs Stellen
über die Temperatur bzw. Zeit stabil bleiben sollen.

http://www.mino-elektronik.de
Alle Angaben wie immer ohne Gewaehr !
2013-09-16
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define BIT(x)  (1<<x)

#define F_CPU           16000000L       // 16MHz ggf. anpassen
#define DEF_BAUDRATE    19200

#define MS_TEILER       (F_CPU/1000)    // Intervall für 1ms

#define STELLEN         6               // angezeigte Stellen
#define MESSZEIT        300             // max. 3 Messungen/Sek. 
#define LED_EIN         MESSZEIT/2      // 50%
#define TIMEOUT         62500           // max. 62,5s warten
#define EE_ADR_OFFSET   2               // Platz für Korrekturwert
#define KAL_TASTE       1               // PORTC-1

enum status {MESSEN=0, AUSLESEN, AUSWERTEN};    // die Phasen der Messung
enum anzeige_typ {FREQUENZ, PERIODE, DREHZAHL}; // zur Auswahl

float frequenz;             // float-Ergebnis
float f_clock;              // ggf. mit Korrekturwert

volatile uint8_t messwert_vorhanden,    // nur gueltige Messungen anzeigen
        mess_status,        // Ablaufsteuerung
        ms10_flag;          // wird mit T1OV gesetzt

volatile uint16_t zeit_low, // Timer1-Anteil
        zeit_high,          // T1 Ueberlauf-Anteil
        mess_dauer,         // minimale Wartezeit
        ueberlauf;

volatile uint32_t start_ereignis,       // Impulse zu Beginn der Messung
        end_ereignis,       // Impulse am Ende der Messung
        start_zeit,         // rel. Zeit: Beginn der Messung
        end_zeit,           // Zeitpunkt der Auswertung
        mess_zeit,          // genaue Zeit der Messung
        mess_ereignisse;    // Impulse der Messung


const uint8_t f_dim[][4]={"GHz", "MHz","kHz","Hz ","mHz",           // Dimension für RS232-Ausgabe
                          "sek","ms ","us ","ns ","ps "};
const uint8_t u_dim[][6]={"Grpm ","Mrpm ","krpm ","rpm ","mrpm "};

uint8_t lese_adc(uint8_t kanal)             // nur 8 Bit verwerten
{
  ADMUX =  BIT(REFS0) | BIT(ADLAR) | kanal; // Vcc als Vref, linksbuendig
  ADCSRA |= BIT(ADSC);                      // starten
  while(ADCSRA & BIT(ADSC));                // warten
  return(ADCH);                             // fertig  
}

uint8_t anzeige_wert(void)                  // Spannung an ADC0 als Meßmodus bewerten
{
uint8_t temp = lese_adc(ADCH0);              
  if(temp >= 230) return(FREQUENZ);         // bei offenem Eingang FREQUENZ
  if(temp <= 20)  return(PERIODE);          // bei KurzschlußŸ PERIODE
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
  TCCR0B = 0x07;                    // T0 zaehlt Impulse des Vorteilers
  TCNT1 = 0;
  OCR1A = MS_TEILER;                // ms-Intervall für Timing
  OCR1B = MS_TEILER/2;              // 0,5ms
  TCCR1A |= BIT(COM1A0);            // togglen mit OCR1B Intervall -> 1kHz Ausgang
  TCCR1B = 0x01;                    // Timer1 ohne Vorteiler
  TIMSK1 = BIT(ICIE1) |             // ICP1 Int-enable Frequenz vom Vorteiler
           BIT(TOIE1) |             // OVFL Int-enable 
           BIT(OCIE1A) |            // COMP1A Int-enable für 1Khz Siganl
           BIT(OCIE1B);             // COMP1B Int-enable für ms timing
  ADCSRA = BIT(ADEN) + 0x07;        // einschalten mit Teiler /128

}

// Routine zur Wandlung und Ausgabe eines Meßwertes

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
  } else { k=0;dimension=3;}                // für x = 0.0

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

// 1kHz Interrupt fÃ¼r ms-genaues Timing
ISR(TIMER1_COMPB_vect)              // wird mit 1000 Hz aufgerufen
{
static uint8_t n;
  OCR1B += MS_TEILER;         
  sei();                            // gleich wieder freigeben
// ab hier unkritisches Timing
  mess_dauer++;                     // ungefaehre Dauer der Messung
  if(n >= 10) {
    n = 0;
    ms10_flag = 1;                  // alle 10ms
  }
}

// Routinen zur Erfassung der Eingangsimpulse
// ÃœberlÃ¤ufe von T1
ISR(TIMER1_OVF_vect)            
{
  ueberlauf++;                  // Ueberlaeufe von T1 ergeben obere 16bit der Messzeit
}

// CAPT-Ereignisse an AIN1
ISR(TIMER1_CAPT_vect)           // Eingangsimpulse mit genauem Zeitpunkt erfassen
{
static unsigned long count;     // Impulse per Interrupt                
  count++;
  if(mess_status == AUSLESEN) { // Ergebnisse synchron zum Eingangsimpuls auslesen+ablegen
    end_ereignis = count;       // Anzahl der Impulse lesen
    zeit_low = ICR1;            // capture-reg lesen: untere 16bit
    zeit_high = ueberlauf;      // dazu die oberen 16bit
    if((TIFR1 & BIT(TOV1)) && (zeit_low < 0x8000))    // evtl. Ueberlauf T1 noch offen?
      zeit_high++;              // nur, wenn capture-int + overflow-int gleichzeitig !
    mess_status = AUSWERTEN;    // Daten fertig fuer Auswertung
  }
  if(TIFR1 & BIT(ICF1)) {       // falls neuer Eingangsimpuls gekommen ist
    count++;                    // gleich hier auswerten
    TIFR1 = BIT(ICF1);          // und flag loeschen
  }
}


uint8_t ee_read_byte(uint16_t adr)
{
  while(EECR & BIT(EEPE));      // abwarten
  EEAR = adr;
  EECR |= BIT(EERE);            // lesebit setzen
  return(EEDR);
}

void ee_write_byte(uint8_t data, uint16_t adr)
{
  if(data != ee_read_byte(adr)) {
    cli();
    EEAR = adr;
    EEDR = data;
    EECR  = BIT(EEMPE);         // master schreibbit setzen, EEPMx löschen
    EECR |= BIT(EEPE);          // schreibbit setzen
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
  if((temp ^ temp_alt) & temp) {                    // Taste wurde gedrückt
    if(frequenz > 0.999 && frequenz < 1.001) {      // Frequenzbereich eingrenzen +/-1000ppm
      f_offset = mess_zeit - F_CPU;                 // Differenz ist-soll ermitteln
      f_clock = (float) (F_CPU + f_offset);         // korrigierter Takt
      ser_string((uint8_t *)("Abgleich fertig"));
      ee_write_word(f_offset, EE_ADR_OFFSET);       // Korrekturwert speichern
      messwert_vorhanden = 0;                       // nächte Messung verwerfen
    }
  }
  temp_alt = temp;                                  // fürs nächste Mal merken
}


// Hauptprogramm
int main(void)
{
  PORTB = BIT(PORTB0);                      // Pullup ICP
  DDRB =  BIT(PORTB1);                      // 1kHz signal an OC1A
  PORTD = BIT(PORTD4) | BIT(PORTD7);        // Pullup T0-Eingang, und AIN1
  PORTC = BIT(ADCH1) | BIT(ADCH0);          // Pullup ADC0+ADC1
  ACSR  = BIT(ACIC) | BIT(ACBG);            // analog Komparator verwenden
 
  mess_status = MESSEN;                     // zuerst messen
  messwert_vorhanden = 0;                   // 1.Ergebnis unterdruecken, da es falsch ist
  f_clock = (float) (F_CPU + ee_read_word(EE_ADR_OFFSET));    // eff. Referenzfrequenz

  init_timer_adc();
  init_serio0();
  sei();

  for(;;) {
    cli();
    if(mess_dauer > TIMEOUT) {              // Messung abbrechen
      mess_dauer = 0;
      sei();
      messwert_vorhanden = 0;               // nächstes Ergebnis verwerfen
    }
    cli();
    if(mess_status == MESSEN  && mess_dauer >= MESSZEIT) {
      mess_status = AUSLESEN;               // Auswertung starten
      mess_dauer = 0;                       // neue Messzeit abwarten
    }
    sei();

    if(mess_status==AUSWERTEN) {
      end_zeit = zeit_high*0x10000 + zeit_low;
      mess_zeit = end_zeit - start_zeit;    // Zeit-Differenz bilden
      start_zeit = end_zeit;                // neue startzeit merken
      mess_ereignisse = end_ereignis - start_ereignis;  // Impuls-Differenz
      start_ereignis = end_ereignis;        // fuers naechste Intervall
      mess_status = MESSEN;                 // neue Messung beginnen
      if(messwert_vorhanden) {              // und gültigen Wert anzeigen
        frequenz = ((float)mess_ereignisse * f_clock) / mess_zeit;  // Frequenz berechnen
        zeige_x(frequenz,anzeige_wert(),0); // als Frequenz, Periode oder Drehzahl anzeigen
        teste_kalibrierung();               // nach gültiger Messung ggf. neu abgleichen
      }
      else messwert_vorhanden = 1;          // sperre wieder aufheben
    }
  }
}


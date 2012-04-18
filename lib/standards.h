#ifndef STANDARDS_H
#define STANDARDS_H

#include <util/delay.h>

void init_USART(void)
{
	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit - senden) einschalten
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00);	//Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
	UBRR0H = 0;				//Highbyte ist 0
	UBRR0L = 103;	//Lowbyte ist 103 (dezimal) -> (Frequenz_in_Hz / (Baudrate * 16)) - 1 <- Quarfrequenz = 16*1000*1000 Hz!!!!
}



void sendchar(unsigned char c)
{
	while(!(UCSR0A & (1<<UDRE0))) //Warten, bis Senden möglich ist
	{
	}
	
	UDR0 = c; //schreibt das Zeichen aus 'c' auf die Schnittstelle
}



void sendUSART(char *s) //*s funktioniert wie eine Art Array - auch bei einem String werden die Zeichen (char) einzeln ausgelesen - und hier dann auf die Sendeschnittstelle übertragen
{
	while(*s)
	{
	sendchar(*s);
	s++;
	}
}


/*### ADC-Ansteuerung ###*/

uint16_t adcwert(uint8_t kanal)
{
	uint16_t wert = 0; //Variable für Ergebnis deklarieren

	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//ADEN aktiviert überhaupt erst den internen ADC-Wandler, ADPS2 bis ADPS0 stellen den verwendeten Prescaler ein, denn die Wandlerfrequenz muss immer zwischen 50 und 200 kHz liegen! Der Prescaler muss bei 16MHz also zwischen 80 und 320 eingestellt werden, als einzige Möglichkeit bleibt hier 128 (=alle auf 1).   

	ADMUX = kanal;
	//ADMUX = (1<<REFS1)|(1<<REFS0); //Einstellen der Referenzspannung auf "extern", also REFS1 und REFS0 auf "0" - daher auskommentierte Zeile
	
	ADCSRA |= (1<<ADSC);	//nach Aktivierung des ADC wird ein "Dummy-Readout" empfohlen, man liest also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen"      
    while(ADCSRA & (1<<ADSC)) {} //auf Abschluss der Konvertierung warten
	wert = ADCW;	//ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten Wandlung nicht übernommen.
 
	/* Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen */
	wert = 0; 
	for(uint8_t i=0; i<4; i++)
	{
		ADCSRA |= (1<<ADSC); 	//eine Wandlung "single conversion" starten
		while(ADCSRA & (1<<ADSC)) {} 	//auf Abschluss der Konvertierung warten
		wert = wert + ADCW;	 //Wandlungsergebnisse aufaddieren
    }
	
	ADCSRA &= ~(1<<ADEN);	//ADC deaktivieren
 
	wert = wert/4;		//Durchschnittswert bilden
 
	return wert;
}

uint8_t button(void)
{
	uint8_t taste = 0; 	//Variable für Nummer des Tasters
	uint16_t analog7 = adcwert(7);	//Wert des Ports
	
	PORTA |= (1<<PA7);		//Ohne das hier "flackern" die Werte aus irgend einem Grund -> es werden mitunter Tasten erkannt, die gar nicht gedrückt wurden oder das Programm bleibt für einige Sekunden "hängen"
	_delay_ms(1);
	PORTA &= ~(1<<PA7);
	
	//Abfrage des gedrückten Tasters - um Störungen zu vermeiden wurden
        //die Bereiche sehr eng gefasst, sollten bei Bedarf an jedes Board extra angepasst werden.
	if((analog7>=337) && (analog7<=343)) {taste = 1;} // angepasst, da dieser Taster zuerst nicht angesprochen hat
	else if((analog7>=268) && (analog7<=274)) {taste = 2;}
	else if((analog7>=200) && (analog7<=206)) {taste = 3;}
	else if((analog7>=132) && (analog7<=138)) {taste = 4;}
	else if((analog7>=64) && (analog7<=70)) {taste = 5;}
	else {}
	
	return taste;
}

#endif
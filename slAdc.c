/*
 * slAdc.c
 *
 *  Created on: 14-04-2016
 *      Author: db
 */
#include <avr/io.h>
#include <stdlib.h>
#include "slAdc.h"



void slADC_init() {
	ADCSRA |= (1 << ADEN); //włącz ADC
	ADCSRA |= PRESCALER_128; //dobry preskaler
	ADMUX |= REF_256; //ustawiamy wewn. źródłó
}

uint16_t slADC_measure(uint8_t pin) {
	ADMUX = (ADMUX & 0xf8); // 0b11111000 Zerujemy trzy starse bity
	ADMUX = ADMUX | pin; // Co jest równoznaczne z zapisem ADMUX = ADMUX | (1<<MUX2) | (1<<MUX0); dla pinu PC5
	//my korzysamy z tego że wartości trzech starsze bitow tego rejestru odpowiadają zapisowi bitowego kolejnych portów
	//rejestru PCn
	ADCSRA |= (1 << ADSC); //start konwersji
	while (ADCSRA & (1 << ADSC)) {
	} //czekamy aż się skończy pomiar
	return ADCW; //zwracamy pomiar (korzystając z wbudowanego makra)
}



void get_vol(uint16_t adc, TVOL * voltage) {
	uint32_t sr1 = 0;
	uint8_t i = 0;
	//voltage->idx = 0;
	voltage->v2[0] = '0';
	voltage->v2[1] = '0';
	voltage->sr[voltage->idx++ & (SR - 1)] = adc; //& (SR-1) = maskowanie

	for (i = 0; i < SR; i++) {
		sr1 = sr1 + voltage->sr[i];
	}

	sr1 = (uint32_t)sr1 / SR;

	voltage->adc_mid = sr1;
	uint32_t wynik = (sr1 * (uint32_t) voltage->ref_v) / voltage->ref_adc;
	div_t divmod = div(wynik, 100);
	itoa(divmod.quot, voltage->v1, 10);
	itoa(divmod.rem, voltage->v2, 10); //reszta z dzielenia
	if (divmod.rem < 10) {
		voltage->v2[0] = '0';
		voltage->v2[1] = divmod.rem + '0'; //plus kod 0 = konwersja na asci
	} else {

	}
	voltage->v1[2] = '0'; //zakończenie znakiem zera jak przystało na porządny string
	voltage->v2[2] = '0'; //zakończenie znakiem zera jak przystało na porządny string
}

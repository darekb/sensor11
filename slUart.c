/*
 * slUart.c
 *
 *  Created on: 12-04-2016
 *      Author: db
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include "slUart.h"

void slUART_SimpleTransmitInit() {
    UBRRH = (uint16_t) (__UBRR >> 8);
    UBRRL = (uint8_t) __UBRR;
    /* Enable transmitter */
    UCSRB = (1 << TXEN);
    //reszta jest domyślnie ustawiona po resecie
}

void slUART_Init() {
    slUART_SimpleTransmitInit();

    //Enable receiver
    UCSRB |= (1 << RXEN);

    //Set frame format: 8data, 1stop bit
    //UCSRC = ((1 << URSEL) | (3 << UCSZ0)) & ~(1 << USBS);
#ifdef URSEL
    UCSRC = (1<<URSEL) | (3<<UCSZ0) & ~(1 << USBS);
#else
    UCSRC = (3 << UCSZ0) & ~(1 << USBS);
#endif

    // (1 << URSEL) bo rejestr UCSRC jest współdzielony i musimy wskazać co zmieniamy
    //taki ficzer ATmegi8

    // | (3 << UCSZ0) UCSZ2=0 UCSZ1=1 UCSZ0=1 czyli transfer 8bitowy

    // & ~(1 << USBS) USBS = 0 jednobitowy koniec ramki
}

void slUART_WriteByte(char data) {
    while (!(UCSRA & (1 << UDRE))) {
    }
    UDR = data;
}

void slUART_WriteString(const char myString[]) {
    uint16_t i = 0;
    while (myString[i]) {
        slUART_WriteByte(myString[i]);
        i++;
    }
}
void slUART_WriteStringNl(const char myString[]) {
    slUART_WriteString(myString);
    slUART_WriteString("\r\n");
}
void slUART_WriteBuffer(const uint8_t myData[], uint8_t length) {
		uint8_t buff[8];
    memcpy(&buff, myData, 8*sizeof(uint8_t));
    for (uint8_t i = 0; i < length;  i++) {
    	slUART_LogHex((uint16_t)buff[i]);
    	slUART_WriteByte(' ');
    }
    slUART_WriteString("\r\n");
}

void slUART_LogBinary(uint16_t dataIn) {
    char buff[30];
    itoa(dataIn, buff, 2);
    slUART_WriteString(buff);
}
void slUART_LogBinaryNl(uint16_t dataIn) {
    slUART_LogBinary(dataIn);
    slUART_WriteString("\r\n");
}

void slUART_LogDec(uint16_t dataIn) {
    char buff[30];
    itoa(dataIn, buff, 10);
    slUART_WriteString(buff);
}
void slUART_LogDecWithSign(int16_t dataIn) {
    char buff[30];
    itoa(dataIn, buff, 10);
    slUART_WriteString(buff);
}
void slUART_LogDecNl(uint16_t dataIn) {
    slUART_LogDec(dataIn);
    slUART_WriteString("\r\n");
}

void slUART_LogHex(uint16_t dataIn) {
    char buff[30];
    itoa(dataIn, buff, 16);
    slUART_WriteString(buff);
}
void slUART_LogHexNl(uint16_t dataIn) {
    slUART_LogHex(dataIn);
    slUART_WriteString("\r\n");
}

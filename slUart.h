/*
 * slUart.h
 *
 *  Created on: 12-04-2016
 *      Author: db
 */

#ifndef SLUART_H_
#define SLUART_H_


// compatibility for the mega161
#ifndef RXCIE
#define RXCIE RXCIE0
#define TXCIE TXCIE0
#define UDRIE UDRIE0
#define RXEN  RXEN0
#define TXEN  TXEN0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define UCSRB UCSR0B
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UDRE  UDRE0
#define UDR   UDR0
#define UCSZ0 UCSZ00
#define USBS  USBS0
#endif

#ifndef F_CPU
#define F_CPU 8000000UL
#endif
//#define UART_BAUD 9600
#define UART_BAUD 38400
//#define UART_BAUD 115200
#define __UBRR ((F_CPU + UART_BAUD * 8UL) / (16UL * UART_BAUD )-1)

void slUART_Init();

void slUART_SimpleTransmitInit();

void slUART_WriteByte(char data);

void slUART_WriteString(const char myString[]);

void slUART_WriteStringNl(const char myString[]);
void slUART_WriteBuffer(const uint8_t myData[], uint8_t length);

void slUART_LogBinary(uint16_t dataIn);
void slUART_LogBinaryNl(uint16_t dataIn);

void slUART_LogDec(uint16_t dataIn);
void slUART_LogDecWithSign(int16_t dataIn);
void slUART_LogDecNl(uint16_t dataIn);

void slUART_LogHex(uint16_t dataIn);
void slUART_LogHexNl(uint16_t dataIn);

#endif /* SLUART_H_ */

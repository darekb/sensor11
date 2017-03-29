//
// Created by dariusz on 20.02.17.
//
#include <avr/io.h>
#include "slSPI.h"
//#include "slUart.h"

void slSPI_Init() {
    //Set MOSI and SCK output, all others input
    PORTB |= (1 >> DD_SS);//high
    DDR_SPI  |= (1 << DD_SS);//output
    //set AVR as master
    slSPI_SetMaster();

    //enable spi interface
    slSPI_Enable();

    DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK);
//    DDR_SPI &= ~(1 << DD_MISO);//input
//    PORTB |= (1 << DD_MISO);//pullup

    //SPI Most Significant Bit First
    slSPI_SetMsb();

    //Mode 0 Rising edge of data, keep clock low
    slSPI_SetMode0();

    //Run the data in at 16MHz/4 - 4MHz
    slSPI_SetClockDiv2();

}

uint8_t slSPI_TransferInt(uint8_t data) {
    /* Start transmission */
//    slUART_WriteString("slSPI_TransferInt data: ");
//    slUART_LogBinary(data);
    SPDR = data;
    asm volatile("nop");
    /* Wait for transmission complete */
    while (!(SPSR & _BV(SPIF))) ;
    return SPDR;
}
//
//void slSPI_WriteString(const char myString[], char *buffOut) {
//    uint8_t i = 0;
//    while (myString[i]) {
//        buffOut[i] = slSPI_TransferInt(myString[i]);
//        i++;
//    }
//}
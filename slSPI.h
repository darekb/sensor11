//
// Created by dariusz on 20.02.17.
//

#ifndef CMAKE_AVR_SPLSPI_H
#define CMAKE_AVR_SPLSPI_H

#define DDR_SPI DDRB
//nano 12
#define DD_MISO PB4
//nano 11
#define DD_MOSI PB3
//nano 13
#define DD_SCK  PB5
//nano 13
#define DD_SS   PB2


// Table 19-5.
// Relationship Between SCK and the Oscillator Frequency
// SPI2X SPR1 SPR0  SCK Frequency
// 0     0    0     f osc / 4
// 0     0    1     f osc / 16
// 0     1    0     f osc / 64
// 0     1    1     f osc / 128
// 1     0    0     f osc / 2
// 1     0    1     f osc / 8
// 1     1    0     f osc / 32
// 1     1    1     f osc / 64


#define slSPI_SetClockDiv2()    SPCR &= ~(1<<SPR1) & ~(1<<SPR0); SPSR |= (1 << SPI2X);
#define slSPI_SetClockDiv4()    SPCR &= ~(1<<SPR1) & ~(1<<SPR0); SPSR &= ~(1 << SPI2X);
#define slSPI_SetClockDiv8()    SPCR |= (1<<SPR0); SPCR &= ~(1<<SPR1); SPSR |= (1 << SPI2X);
#define slSPI_SetClockDiv16()   SPCR |= (1<<SPR0); SPCR &= ~(1<<SPR1); SPSR &= ~(1 << SPI2X);
#define slSPI_SetClockDiv32()   SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0); SPSR |= (1 << SPI2X);
#define slSPI_SetClockDiv64()   SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0); SPSR &= ~(1 << SPI2X);
#define slSPI_SetClockDiv128()  SPCR |= (1<<SPR1) | (1<<SPR0); SPSR |= (1 << SPI2X);


#define slSPI_SetLsb()          SPCR |= _BV(DORD)
#define slSPI_SetMsb()          SPCR &= ~(_BV(DORD))

#define slSPI_SetMaster()       SPCR |= _BV(MSTR)
#define slSPI_SetSlave()        SPCR &= ~(_BV(MSTR))

#define slSPI_Enable()          SPCR |= _BV(SPE)
#define slSPI_Disable()         SPCR &= ~(_BV(SPE));


//CPOL = bit 3, CPHA = bit 2 on SPCR
#define slSPI_SetMode0()        SPCR &= ~(1<<CPOL) & ~(1<<CPHA)
#define slSPI_SetMode1()        SPCR |= (1<<CPHA); SPCR &= ~(1<<CPOL)
#define slSPI_SetMode2()        SPCR |= (1<<CPOL); SPCR &= ~(1<<CPHA)
#define slSPI_SetMode3()        SPCR |= (1<<CPOL) | (1<<CPHA)

void slSPI_Init();
uint8_t slSPI_TransferInt(uint8_t data);
//void slSPI_WriteString(const char myString[], char *buffOut);
#endif //CMAKE_AVR_SPLSPI_H

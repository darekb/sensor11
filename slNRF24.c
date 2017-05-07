/*
 * RF_Tranceiver.c
 *
 * Created: 2012-08-10 15:24:35
 *  Author: Kalle
 *  Atmega88
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "slNRF24.h"
#include "slSPI.h"


void slNRF24_IoInit(void) {
    CSN_OUTPUT();
    CE_OUTPUT();
    CSN_HIGH();
    CE_LOW();
}


void slNRF24_SetRegister(uint8_t reg, void *dataIn, uint8_t len) {
    uint8_t *data = dataIn;
    //cli();
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(W_REGISTER + reg);
    _delay_us(10);
    for (uint8_t i = 0; i < len; i++) {
        slSPI_TransferInt(data[i]);
        _delay_us(10);
    }
    CSN_HIGH();
    //sei();
}

void slNRF24_GetRegister(uint8_t reg, uint8_t *dataIn, uint8_t len) {
    //cli();
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(R_REGISTER + reg);
    _delay_us(10);
    for (uint8_t i = 0; i < len; i++) {
        dataIn[i] = slSPI_TransferInt(0x00);
        _delay_us(10);
    }
    CSN_HIGH();
    //sei();
}


//initierar nrf'en (obs nrfen måste vala i vila när detta sker CE-låg)
void slNRF24_Init(void) {
    _delay_ms(100);    //allow radio to reach power down if shut down
    uint8_t val[5];

    //SETUP_RETR (the setup for "EN_AA")
    val[0] = 0x3F;    //0b0010 00011 "2" sets it up to 750uS delay between every retry (at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps) "F" is number of retries (1-15, now 15)
    slNRF24_SetRegister(SETUP_RETR, val, 1);

    //Enable ‘Auto Acknowledgment’ Function on data pipe 0 and pipe 1
    val[0] = 0x3f;
    slNRF24_SetRegister(EN_AA, val, 1);

    //enable data pipe 1 for RX
    val[0] = 0x3f;
    slNRF24_SetRegister(EN_RXADDR, val, 1);

    //Setup of Address Widths 5 bytes
    val[0] = 0x03;
    slNRF24_SetRegister(SETUP_AW, val, 1);

    //RF channel setup - 2,400-2,527GHz 1MHz/chanel
    val[0] = 0x1;//2,401Ghz
    slNRF24_SetRegister(RF_CH, val, 1);

    //RF setup	- 2Mbps spped and 0dBm
    val[0] = 0x0e;
    slNRF24_SetRegister(RF_SETUP, val, 1);

    slNRF24_ChangeAddress(SENSOR_ADDR);

    val[0] = 9;
    slNRF24_SetRegister(RX_PW_P0, val, 1);
    slNRF24_SetRegister(RX_PW_P1, val, 1);

    //CONFIG reg setup - Mask interrupt caused by MAX_RT disabled enable CRC CRC 2 byte scheme power up
    val[0] = 0x1E;
    slNRF24_SetRegister(CONFIG, val, 1);


    //device need 1.5ms to reach standby mode
    _delay_ms(100);

}

void slNRF24_ChangeAddress(uint8_t adress) {
    _delay_ms(100);
    uint8_t val[5];
    for (uint8_t i = 0; i < 5; i++) {
        val[i] = adress;
    }
    slNRF24_SetRegister(RX_ADDR_P0, val, 5);
    slNRF24_SetRegister(TX_ADDR, val, 5);
    // for(uint8_t i=0; i<5; i++){
    //     val[i]=0xAA;
    // }
    slNRF24_SetRegister(RX_ADDR_P1, val, 5);
    // for(uint8_t i=0; i<5; i++){
    //     val[i]=adress;
    // }
    // slNRF24_SetRegister(RX_ADDR_P0, val, 5); 
    // slNRF 24_SetRegister(TX_ADDR, val, 5);
    // slNRF24_SetRegister(RX_ADDR_P1, val, 5);
    _delay_ms(100);
}

void slNRF24_Reset(void) {
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(W_REGISTER + STATUS);
    _delay_us(10);
    //reset Data Ready RX FIFO interrupt, Data Sent TX FIFO interrupt, Maximum number of TX retransmits interrupt 
    slSPI_TransferInt(0x70);
    _delay_us(10);
    CSN_HIGH();
}

//Sänd data
void slNRF24_TransmitPayload(void *dataIn, uint8_t len) {
    uint8_t *data = dataIn;

    cli();
    slNRF24_SetRegister(RX_PW_P0, &len, 1);
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(W_TX_PAYLOAD);
    _delay_us(10);
    for (uint8_t i = 0; i < len; i++) {
        slSPI_TransferInt(data[i]);
        _delay_us(10);
    }
    CSN_HIGH();
    _delay_ms(10);
    CE_HIGH();
    _delay_us(20);
    CE_LOW();
    _delay_ms(10);
    sei();
    //cli();	//Disable global interrupt...

}


void slNRF24_TxPowerUp() {
    uint8_t configReg;
    slNRF24_GetRegister(CONFIG, &configReg, 1);
    configReg |= (1 << PWR_UP);
    configReg &= ~(1 << PRIM_RX);
    slNRF24_SetRegister(CONFIG, &configReg, 1);
    CE_LOW();
    //slNRF24_ChangeAddress(SENSOR_ADDR);
}

void slNRF24_RxPowerUp() {
    uint8_t configReg;
    slNRF24_GetRegister(CONFIG, &configReg, 1);
    configReg |= (1 << PWR_UP) | (1 << PRIM_RX);
    slNRF24_SetRegister(CONFIG, &configReg, 1);
    CE_HIGH();
    //slNRF24_ChangeAddress(SENSOR_ADDR);
    //slNRF24_FlushTx();
}

void slNRF24_PowerDown() {
    uint8_t configReg;
    slNRF24_GetRegister(CONFIG, &configReg, 1);
    configReg |= (1 << PRIM_RX);
    configReg &= ~(0 << PWR_UP);
    slNRF24_SetRegister(CONFIG, &configReg, 1);
    CE_LOW();
}


void slNRF24_FlushTx() {
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(FLUSH_TX);
    _delay_us(10);
    CSN_HIGH();
}

void slNRF24_FlushRx() {
    _delay_us(10);
    CSN_LOW();
    _delay_us(10);
    slSPI_TransferInt(FLUSH_RX);
    _delay_us(10);
    CSN_HIGH();
}

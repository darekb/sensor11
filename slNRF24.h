//
// Created by dariusz on 20.02.17.
//

#ifndef CMAKE_AVR_SLNRF24_H
#define CMAKE_AVR_SLNRF24_H

/* Bit Mnemonics */

#define RX_P_NO     1
#define TX_FULL     0

#define CE_PIN        PB1
#define CE_OUTPUT()   DDRB |= (1 << CE_PIN)
#define CE_HIGH()     PORTB |= (1 << CE_PIN)//enable RX
#define CE_LOW()      PORTB &= ~(1 << CE_PIN)//disable RX
#define CSN_PIN       PB2
#define CSN_OUTPUT()  DDRB |= (1 << CSN_PIN)
#define CSN_HIGH()    PORTB |= (1 << CSN_PIN)
#define CSN_LOW()     PORTB &= ~(1 << CSN_PIN)

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

#define NRF_SETTINGS (1 << MASK_TX_DS) | (1 << MASK_MAX_RT) | (1 << EN_CRC)
#define NRF_PAYLOAD_SIZE 4
#define NRF_ADDR_LEN 5

#define slNRF_CONFIG      0x00
#define slNRF_EN_AA       0x01
#define slNRF_RF_CH       0x05
#define slNRF_RF_SETUP    0x06
#define slNRF_RPD         0x09



// SPI Commands

#define R_REGISTER          0x00 // 000A AAAA
#define W_REGISTER          0x20 // 001A AAAA
#define R_RX_PAYLOAD        0x61 // 0110 0001
#define W_TX_PAYLOAD        0xA0 // 1010 0000
#define FLUSH_TX            0xE1 // 1110 0001
#define FLUSH_RX            0xE2 // 1110 0010
#define REUSE_TX_PL         0xE3 // 1110 0011
#define R_RX_PL_WID         0x60 // 0110 0000
#define W_ACK_PAYLOAD       0xA8 // 1010 1PPP
#define W_TX_PAYLOAD_NOACK  0xB0 // 1011 0000
#define NOP                 0xFF // 1111 1111


// Register Map

#define CONFIG      0x00
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define EN_AA       0x01
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define EN_RXADDR   0x02
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

#define SETUP_AW    0x03
#define AW          0
#define SETUP_AW_3  0x01 //3 bytes
#define SETUP_AW_4  0x02 //4 bytes
#define SETUP_AW_5  0x03 //5 bytes

#define SETUP_RETR  0x04
#define ARD         4
#define ARC         0

#define RF_CH       0x05

#define RF_SETUP    0x06
#define CONT_WAVE   7
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR      1

#define STATUS              0x07
#define RX_DR               6
#define TX_DS               5
#define MAX_RT              4
#define RX_P_NO_MASK        0x0E
#define STATUS_TX_FULL      0

#define OBSERVE_TX  0x08
#define PLOS_CNT    4
#define ARC_CNT     0

#define RPD         0x09

#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F

#define TX_ADDR     0x10

#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16

#define FIFO_STATUS         0x17
#define TX_REUSE            6
#define FIFO_TX_FULL        5
#define TX_EMPTY            4
#define RX_FULL             1
#define RX_EMPTY            0

#define DYNPD       0x1C
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0

#define FEATURE     0x1D
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

//#define RF24_1MBPS   0
//#define RF24_2MBPS   1
//#define RF24_250KBPS 2

//#define RF24_PA_MIN   0
//#define RF24_PA_LOW   1
//#define RF24_PA_HIGH  2
//#define RF24_PA_MAX   3
//#define RF24_PA_ERROR 4

#define PRIPSTR "%S"


/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;


#define ADDRESS_WIDTH 5//

void slNRF_Init();

void slNRF_SetRXPayload(uint8_t pipe, uint8_t bytes);

uint8_t slNRF_GetRegister(uint8_t address, uint8_t log);
uint8_t slNRF_SetRegister(uint8_t address, uint8_t value);
#if showDebugDataNRF24
void slNRF_BitWrite(uint8_t address, uint8_t bit_add, uint8_t val);
#endif

void slNRF_OpenWritingPipe(uint8_t address[], uint8_t payloadSize);
void slNRF_OpenReadingPipe(uint8_t address[], uint8_t payloadSize, uint8_t pipeNr);
void closeReadingPipe(uint8_t pipe);
uint8_t slNRF_SetDataRate(uint8_t dataRateValue);
void slNRF_SetPALevel(uint8_t  paValue ) ;
void slNRF_SetChannel(uint8_t channel);
void slNRF_EnableDynamicPayloads();
void slNRF_DisableDynamicPayloads();
void slNRF_EnableAckPayload();
void slNRF_SetRetries(uint8_t delay, uint8_t countOfTray);
void slNRF_AutoAck(uint8_t isOn);
#if showDebugDataNRF24
void slNRF_showDebugData();
#endif
void slNRF_PowerUp();
void slNRF_PowerDown();
void slNRF_FlushTX();
void slNRF_FlushRX();
void slNRF_StartListening();
void slNRF_StopListening();
uint8_t slNRF_Sent(const uint8_t buf[], uint8_t len);
void slNRF_Recive(uint8_t* buf, uint8_t len );
uint8_t slNRF_Available();

#endif //CMAKE_AVR_SLNRF24_H

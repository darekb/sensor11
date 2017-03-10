//
// Created by dariusz on 20.02.17.
//

#define showDebugDataNRF24 0


#include <avr/io.h>
#include <util/delay.h>
#if showDebugDataNRF24
#include <stdlib.h>
#include <stdio.h>
#endif
#include <avr/pgmspace.h>
#include "slNRF24.h"
#include "slSPI.h"

#if showDebugDataNRF24
#include "slUart.h"
#endif



uint8_t dataIn[10];
uint8_t addressWidth = ADDRESS_WIDTH;
uint8_t txDelay = 155;
uint8_t dynamicPayloadsEnabled = 0;
uint8_t pipe0ReadingAddress[ADDRESS_WIDTH];
uint8_t payloadWidth = 9;

static const uint8_t childPipeEnable[] PROGMEM =
        {
                ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
        };
static const uint8_t childPipe[] PROGMEM =
        {
                RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
        };
static const uint8_t childPayloadSize[] PROGMEM =
        {
                RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
        };


void delay_us(uint8_t count) {
    while (count--) {
        _delay_us(1);
    }
}
#if showDebugDataNRF24
uint8_t bitRead(uint8_t dataIn, uint8_t x) {
    if (bit_is_set(dataIn, x)) {
        return 1;
    } else {
        return 0;
    }
}
#endif
//
void returnData(uint8_t address) {
#if showDebugDataNRF24
    switch (address) {
        case 0:
            slUART_WriteString(" CONFIG REGISTER =");
            slUART_LogBinaryNl(dataIn[1]);
            slUART_WriteString("PRIM_RX = ");
            if (bitRead(dataIn[1], 0))
                slUART_WriteStringNl("PRX");
            else
                slUART_WriteStringNl("PTX");

            slUART_WriteString("PWR_UP = ");
            if (bitRead(dataIn[1], 1))
                slUART_WriteStringNl("POWER UP");
            else
                slUART_WriteStringNl("POWER DOWN");

            slUART_WriteString("CRCO = ");
            if (bitRead(dataIn[1], 2))
                slUART_WriteStringNl("2Bytes");
            else
                slUART_WriteStringNl("1Byte");

            slUART_WriteString("EN_CRC = ");
            if (bitRead(dataIn[1], 3))
                slUART_WriteStringNl("Enabled");
            else
                slUART_WriteStringNl("Disabled");

            slUART_WriteString("MASK_MAX_RT = ");
            if (bitRead(dataIn[1], 4))
                slUART_WriteStringNl("Interrupt not reflected on the IRQ pin");
            else
                slUART_WriteStringNl("Reflect MAX_RT as active low interrupt on the IRQ pin");

            slUART_WriteString("MASK_TX_DS = ");
            if (bitRead(dataIn[1], 5))
                slUART_WriteStringNl("Interrupt not reflected on the IRQ pin");
            else
                slUART_WriteStringNl("Reflect TX_DS as active low interrupt on the IRQ pin");

            slUART_WriteString("MASK_RX_DR = ");
            if (bitRead(dataIn[1], 6))
                slUART_WriteStringNl("Interrupt not reflected on the IRQ pin");
            else
                slUART_WriteStringNl("Reflect RX_DR as active low interrupt on the IRQ pin");
            break;//0
        case 1:
            slUART_WriteString(" EN_AA REGISTER Enhanced ShockBurst =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//1
        case 2:
            slUART_WriteString(" EN_RXADDR REGISTER Enabled RX Addresses =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//2
        case 3:
            slUART_WriteString(" SETUP_AW REGISTER Setup of Address Widths =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//3
        case 4:
            slUART_WriteString(" SETUP_RETR REGISTER Setup of Automatic Retransmission =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//4
        case 5:
            slUART_WriteString(" RF_CH REGISTER RF Channel =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//5
        case 6:
            slUART_WriteString(" RF_SETUP REGISTER RF Setup Register =");
            slUART_LogBinaryNl(dataIn[1]);
            slUART_WriteString("RF Power = ");
            slUART_LogBinary(bitRead(dataIn[1], 2));
            slUART_LogBinaryNl(bitRead(dataIn[1], 1));
            slUART_WriteString("RF_DR_HIGH = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 3));
            slUART_WriteString("PLL_LOCK = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 4));
            slUART_WriteString("RF_DR_LOW = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 5));
            slUART_WriteString("CONT_WAVE = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 7));
            break;//6
        case 7:
            slUART_WriteString(" STATUS REGISTER  =");
            slUART_LogBinaryNl(dataIn[1]);
            slUART_WriteString("TX_FULL = ");
            if (bitRead(dataIn[1], 0))
                slUART_WriteStringNl("TX FIFO full");
            else
                slUART_WriteStringNl("TX FIFO Not full");

            slUART_WriteString("RX_P_NO = ");
            if (bitRead(dataIn[1], 1) && bitRead(dataIn[1], 2) && bitRead(dataIn[1], 3))
                slUART_WriteStringNl("RX FIFO Empty");
            else
                slUART_LogBinaryNl(bitRead(dataIn[1], 1) + (bitRead(dataIn[1], 2) << 1) + (bitRead(dataIn[1], 2) << 2));
            slUART_WriteString("MAX_RT Interrupt = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 4));
            slUART_WriteString("TX_DS Interrupt = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 5));
            slUART_WriteString("RX_DR Interrupt = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 6));
            break;//7
        case 8:
            slUART_WriteString(" OBSERVE_TX REGISTER Transmit observe register  =");
            slUART_LogBinaryNl(dataIn[1]);
            slUART_WriteString("ARC_CNT = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 0) + (bitRead(dataIn[1], 1) << 1) + (bitRead(dataIn[1], 2) << 2) +
                               (bitRead(dataIn[1], 3) << 3));
            slUART_WriteString("PLOS_CNT = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 4) + (bitRead(dataIn[1], 5) << 1) + (bitRead(dataIn[1], 6) << 2) +
                               (bitRead(dataIn[1], 7) << 3));
            break;//8
        case 9:
            slUART_WriteString(" RPD REGISTER Received Power Detector =");
            slUART_LogBinaryNl(bitRead(dataIn[1], 0));
            break;//9
        case 10:
            slUART_WriteString(" RX_ADDR_P0 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//10
        case 11:
            slUART_WriteString(" RX_ADDR_P1 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//11
        case 12:
            slUART_WriteString(" RX_ADDR_P2 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//12
        case 13:
            slUART_WriteString(" RX_ADDR_P3 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//13
        case 14:
            slUART_WriteString(" RX_ADDR_P4 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//14
        case 15:
            slUART_WriteString(" RX_ADDR_P5 LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//15
        case 16:
            slUART_WriteString(" TX_ADDR LSB =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//16
        case 17:
            slUART_WriteString(" RX_PW_P0 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//17
        case 18:
            slUART_WriteString(" RX_PW_P1 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//18
        case 19:
            slUART_WriteString(" RX_PW_P2 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//19
        case 20:
            slUART_WriteString(" RX_PW_P3 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//20
        case 21:
            slUART_WriteString(" RX_PW_P4 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//21
        case 22:
            slUART_WriteString(" RX_PW_P5 RX payload =");
            slUART_LogBinaryNl(dataIn[1]);
            break;//22

        case 23:
            slUART_WriteString(" FIFO_STATUS Register =");
            slUART_LogBinaryNl(dataIn[1]);
            slUART_WriteString("RX_EMPTY = ");
            if (bitRead(dataIn[1], 0))
                slUART_WriteStringNl("RX FIFO empty");
            else
                slUART_WriteStringNl("Data in RX FIFO");

            slUART_WriteString("RX_EMPTY = ");
            if (bitRead(dataIn[1], 1))
                slUART_WriteStringNl("RX FIFO full");
            else
                slUART_WriteStringNl("Available locations in RX FIFO");

            slUART_WriteString("TX_EMPTY = ");
            if (bitRead(dataIn[1], 4))
                slUART_WriteStringNl("TX FIFO empty");
            else
                slUART_WriteStringNl("Data in TX FIFO");

            slUART_WriteString("TX_FULL = ");
            if (bitRead(dataIn[1], 5))
                slUART_WriteStringNl("TX FIFO full");
            else
                slUART_WriteStringNl("Available locations in TX FIFO");
            slUART_WriteString("TX_REUSE = ");
            slUART_LogBinaryNl(bitRead(dataIn[1], 6));
            break;//23
    }//switch
#endif
}

void slNRF_Init() {

#if showDebugDataNRF24
    slUART_WriteString("after init SPI\r\n\r\n");
#endif
    slSPI_Init();
    CE_OUTPUT();
    CSN_OUTPUT();
    CSN_HIGH();
    CE_HIGH();
    CE_LOW();
    //how long RX/TX addres is (now 5 bytes) (addressWidth)
    slNRF_SetRegister(SETUP_AW, SETUP_AW_5);
}

uint8_t slNRF_GetRegister(uint8_t address, uint8_t log) {
    CSN_LOW();
    dataIn[0] = slSPI_TransferInt(address);
    dataIn[1] = slSPI_TransferInt(0x00);
    CSN_HIGH();
    if (log == 1) {
        returnData(address);
    }
    return dataIn[1];
}

uint8_t slNRF_SetRegister(uint8_t address, uint8_t value) {
    CSN_LOW();
    dataIn[0] = slSPI_TransferInt(W_REGISTER | address);
    dataIn[1] = slSPI_TransferInt(value);
    CSN_HIGH();
    return dataIn[1];
}
uint8_t slNRF_ReadRegister(uint8_t reg, uint8_t* buf, uint8_t len){
    uint8_t status;
    CSN_LOW();
    status = slSPI_TransferInt( R_REGISTER | reg );
    while ( len-- ){
        *buf++ = slSPI_TransferInt(0xff);
    }
    CSN_HIGH();

    return status;
}

void slNRF_SendCommand(uint8_t address, void *value, uint8_t length) {
    CSN_LOW();
    dataIn[0] = slSPI_TransferInt(W_REGISTER | address);
    for (uint8_t i = 0; i < length; i++) {
        dataIn[(i + 1)] = slSPI_TransferInt(((uint8_t *) value)[i]);
    }
    CSN_HIGH();
}

void slNRF_OpenWritingPipe(uint8_t address[], uint8_t payloadSize) {
#if showDebugDataNRF24
     slNRF_GetRegister(pgm_read_byte(&childPipe[0]), 1);
     slNRF_GetRegister(TX_ADDR, 1);
     slNRF_GetRegister(pgm_read_byte(&childPayloadSize[0]), 1);
#endif

    slNRF_SendCommand(pgm_read_byte(&childPipe[0]), address, addressWidth);
    slNRF_SendCommand(TX_ADDR, address, addressWidth);
    slNRF_SetRegister(pgm_read_byte(&childPayloadSize[0]), payloadSize);
#if showDebugDataNRF24
     slNRF_GetRegister(pgm_read_byte(&childPipe[0]), 1);
     slNRF_GetRegister(TX_ADDR, 1);
     slNRF_GetRegister(pgm_read_byte(&childPayloadSize[0]), 1);
#endif
}

void slNRF_OpenReadingPipe(uint8_t address[], uint8_t payloadSize, uint8_t pipeNr) {
#if showDebugDataNRF24
     slNRF_GetRegister(pgm_read_byte(&childPipe[pipeNr]), 1);
     slNRF_GetRegister(pgm_read_byte(&childPayloadSize[pipeNr]), 1);
     slNRF_GetRegister(EN_RXADDR, 1);
#endif
    for (uint8_t i = 0; i < addressWidth; i++) {
        pipe0ReadingAddress[(i + 1)] = slSPI_TransferInt(((uint8_t *) address)[i]);
    }
    slNRF_SendCommand(pgm_read_byte(&childPipe[pipeNr]), address, addressWidth);
    slNRF_SetRegister(pgm_read_byte(&childPayloadSize[pipeNr]), payloadSize);
    slNRF_SetRegister(EN_RXADDR, slNRF_GetRegister(EN_RXADDR, 0) | _BV(pgm_read_byte(&childPipeEnable[pipeNr])));
#if showDebugDataNRF24
     slNRF_GetRegister(pgm_read_byte(&childPipe[pipeNr]), 1);
     slNRF_GetRegister(pgm_read_byte(&childPayloadSize[pipeNr]), 1);
     slNRF_GetRegister(EN_RXADDR, 1);
#endif
}

void closeReadingPipe(uint8_t pipeNr) {
    slNRF_SetRegister(EN_RXADDR, slNRF_GetRegister(EN_RXADDR, 0) & ~_BV(pgm_read_byte(&childPipeEnable[pipeNr])));
}

uint8_t slNRF_SetDataRate(uint8_t dataRateValue) {
    uint8_t result = 0;
    uint8_t setup = slNRF_GetRegister(RF_SETUP, 0);
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    txDelay = 85;
    if (dataRateValue == RF24_250KBPS) {
        txDelay = 155;
    } else {
        if (dataRateValue == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
            txDelay = 65;
        }
    }
    slNRF_SetRegister(RF_SETUP, setup);
    if (slNRF_GetRegister(RF_SETUP, 0) == setup) {
        result = 1;
    }
    return result;
}

void slNRF_SetPALevel(uint8_t paValue) {
    uint8_t setup = slNRF_GetRegister(RF_SETUP, 0) & 0xf8;//0b11111000;
    if (paValue > 3) {              // If invalid level, go to max PA
        paValue = (RF24_PA_MAX << 1) + 1;   // +1 to support the SI24R1 chip extra bit
    } else {
        paValue = (paValue << 1) + 1;     // Else set level as requested
    }
    slNRF_SetRegister(RF_SETUP, setup |= paValue);  // Write it to the chip
}

void slNRF_SetChannel(uint8_t channel) {
    const uint8_t max_channel = 125;
#if showDebugDataNRF24
    slNRF_GetRegister(RF_CH, 1);
#endif
    slNRF_SetRegister(RF_CH, rf24_min(channel, max_channel));
#if showDebugDataNRF24
    slNRF_GetRegister(RF_CH, 1);
#endif
}

void slNRF_EnableDynamicPayloads() {
    slNRF_SetRegister(FEATURE, slNRF_GetRegister(FEATURE, 0) | _BV(EN_DPL));
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    slNRF_SetRegister(DYNPD, slNRF_GetRegister(DYNPD, 1) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) |
                             _BV(DPL_P1) | _BV(DPL_P0));
    dynamicPayloadsEnabled = 1;
}

void slNRF_DisableDynamicPayloads() {
    slNRF_SetRegister(FEATURE, slNRF_GetRegister(FEATURE, 0) & ~_BV(EN_DPL));
    slNRF_SetRegister(DYNPD,0);
    dynamicPayloadsEnabled = 0;
}



void slNRF_EnableAckPayload() {
    // enable ack payload and dynamic payload features
    //toggle_features();
    slNRF_SetRegister(FEATURE, slNRF_GetRegister(FEATURE, 0) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

    // Enable dynamic payload on pipes 0 & 1
    // slNRF_SetRegister(DYNPD, slNRF_GetRegister(DYNPD, 1) | _BV(DPL_P1) | _BV(DPL_P0));
    // dynamicPayloadsEnabled = 1;
}

void slNRF_SetRetries(uint8_t delay, uint8_t countOfTray) {
#if showDebugDataNRF24
    slNRF_GetRegister(SETUP_RETR, 1);
#endif
    slNRF_SetRegister(SETUP_RETR, (delay & 0xf) << ARD | (countOfTray & 0xf) << ARC);
#if showDebugDataNRF24
    slNRF_GetRegister(SETUP_RETR, 1);
#endif
}

void slNRF_AutoAck(uint8_t isOn) {
    if (isOn)
        slNRF_SetRegister(EN_AA, 0x3f);//0b111111
    else
        slNRF_SetRegister(EN_AA, 0);
}
#if showDebugDataNRF24
void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
    char buf[150];

    sprintf_P(buf,PSTR(PRIPSTR"\t ="),name);
    slUART_WriteString(buf);
    while (qty--)
    {
        uint8_t buffer[addressWidth];
        slNRF_ReadRegister(reg++,buffer,sizeof buffer);

        sprintf_P(buf, PSTR(" 0x"));
        slUART_WriteString(buf);
        uint8_t* bufptr = buffer + sizeof buffer;
        while( --bufptr >= buffer )
            sprintf_P(buf, PSTR("%02x"),*bufptr);
            slUART_WriteString(buf);
    }

    //sprintf_P(buf,PSTR("\r\n"));
    slUART_WriteString(PSTR("\r\n"));
}

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
        rf24_datarate_e_str_0,
        rf24_datarate_e_str_1,
        rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
        rf24_model_e_str_0,
        rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
        rf24_crclength_e_str_0,
        rf24_crclength_e_str_1,
        rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = {
        rf24_pa_dbm_e_str_0,
        rf24_pa_dbm_e_str_1,
        rf24_pa_dbm_e_str_2,
        rf24_pa_dbm_e_str_3,
};

rf24_datarate_e getDataRate( void )
{
    rf24_datarate_e result ;
    uint8_t dr = slNRF_GetRegister(RF_SETUP, 0) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if ( dr == _BV(RF_DR_LOW) )
    {
        // '10' = 250KBPS
        result = RF24_250KBPS ;
    }
    else if ( dr == _BV(RF_DR_HIGH) )
    {
        // '01' = 2MBPS
        result = RF24_2MBPS ;
    }
    else
    {
        // '00' = 1MBPS
        result = RF24_1MBPS ;
    }
    return result ;
}

uint8_t getPALevel(void)
{

    return (slNRF_GetRegister(RF_SETUP, 0) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

rf24_crclength_e getCRCLength(void)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;

    uint8_t config = slNRF_GetRegister(CONFIG, 0) & ( _BV(CRCO) | _BV(EN_CRC)) ;
    uint8_t AA = slNRF_GetRegister(EN_AA, 0);

    if ( config & _BV(EN_CRC ) || AA)
    {
        if ( config & _BV(CRCO) )
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }

    return result;
}
void slNRF_showDebugData() {
    uint8_t status;
    status = slSPI_TransferInt(0);
    char buf[150];
    sprintf_P(buf, PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
            status,
            (status & _BV(RX_DR)) ? 1 : 0,
            (status & _BV(TX_DS)) ? 1 : 0,
            (status & _BV(MAX_RT)) ? 1 : 0,
            ((status >> RX_P_NO) & 0x7),
            (status & _BV(TX_FULL)) ? 1 : 0
    );
    slUART_WriteString(buf);

    print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,5);
//    print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
//    print_address_register(PSTR("TX_ADDR\t"),TX_ADDR);
//
//    print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
//    print_byte_register(PSTR("EN_AA\t"),EN_AA);
//    print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR);
//    print_byte_register(PSTR("RF_CH\t"),RF_CH);
//    print_byte_register(PSTR("RF_SETUP"),RF_SETUP);
//    print_byte_register(PSTR("CONFIG\t"),NRF_CONFIG);
//    print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);

//    sprintf_P(buf, PSTR("Data Rate\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
//    slUART_WriteString(buf);
//    //printf_P(PSTR("Model\t\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
//    sprintf_P(buf, PSTR("CRC Length\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
//    slUART_WriteString(buf);
//    sprintf_P(buf, PSTR("PA Power\t = " PRIPSTR "\r\n"),  pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
//    slUART_WriteString(buf);

}
#endif

void slNRF_PowerUp() {
    uint8_t cfg = slNRF_GetRegister(CONFIG, 0);

    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        slNRF_SetRegister(CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        _delay_ms(5);
    }
}
void slNRF_PowerDown() {
    uint8_t cfg = slNRF_GetRegister(CONFIG, 0);
    CE_LOW();
    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        slNRF_SetRegister(CONFIG, cfg & ~_BV(PWR_UP));
    }
}

void slNRF_StartListening() {
    slNRF_SetRegister(CONFIG, slNRF_GetRegister(CONFIG, 0) | _BV(PRIM_RX));
    slNRF_SetRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    CE_HIGH();
    // Restore the pipe0 adddress, if exists
    if (pipe0ReadingAddress[0] > 0) {
        slNRF_SendCommand(RX_ADDR_P0, pipe0ReadingAddress, addressWidth);
    } else {
        closeReadingPipe(0);
    }

    // Flush buffers
    //flush_rx();
    if (slNRF_GetRegister(FEATURE, 0) & _BV(EN_ACK_PAY)) {
        slNRF_FlushTX();
    }
}

void slNRF_StopListening() {
    CE_LOW();

    delay_us(txDelay);

    if (slNRF_GetRegister(FEATURE, 0) & _BV(EN_ACK_PAY)) {
        delay_us(txDelay); //200
        slNRF_FlushTX();
    }
    //flush_rx();
    slNRF_SetRegister(CONFIG, (slNRF_GetRegister(CONFIG, 0)) & ~_BV(PRIM_RX));
    slNRF_SetRegister(EN_RXADDR,
                      slNRF_GetRegister(EN_RXADDR, 0) | _BV(pgm_read_byte(&childPipeEnable[0]))); // Enable RX on pipe0
}

void slNRF_FlushTX() {
    CSN_LOW();
    dataIn[0] = slSPI_TransferInt(FLUSH_TX);
    CSN_HIGH();
}
void slNRF_FlushRX() {
    CSN_LOW();
    dataIn[0] = slSPI_TransferInt(FLUSH_RX);
    CSN_HIGH();
}


uint8_t slNRF_WritePayload(const uint8_t buf[], uint8_t data_len, const uint8_t writeType) {
    uint8_t status;

    // data_len = rf24_min(data_len, payloadWidth);
    // uint8_t blank_len = dynamicPayloadsEnabled ? 0 : payloadWidth - data_len;

    CSN_LOW();
    status = slSPI_TransferInt(writeType);
    for (uint8_t i = 0; i < data_len; i++) {
        slSPI_TransferInt(buf[i]);
    }
    // while (blank_len--) {
    //     slSPI_TransferInt(0);
    // }
    CSN_HIGH();
    return status;
}

uint8_t slNRF_Sent(const uint8_t buf[], uint8_t len) {
    CE_LOW();
    slNRF_WritePayload(buf, len, W_TX_PAYLOAD);
    CE_HIGH();

    while (!(slSPI_TransferInt(0) & (_BV(TX_DS) | _BV(MAX_RT)))) {
    }

    CE_LOW();

    uint8_t status = slNRF_SetRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    //Max retries exceeded
    if (status & _BV(MAX_RT)) {
        slNRF_FlushTX(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
        return 0;
    }
    CSN_LOW();
    slSPI_TransferInt(FLUSH_TX);
    CSN_HIGH();
    //TX OK 1 or 0
    return 1;
    // Make sure the nrf isn't already transmitting
}

uint8_t slNRF_ReadPayload(uint8_t *buf, uint8_t data_len) {
    uint8_t status;

    // if(data_len > 4) data_len = 4;
    // uint8_t blank_len = dynamicPayloadsEnabled ? 0 : 4 - data_len;
    CSN_LOW();
    status = slSPI_TransferInt(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < data_len; i++) {
        buf[i] = slSPI_TransferInt(0xFF);
    }
    // while ( blank_len-- ) {
    //   slSPI_TransferInt(0xff);
    // }
    CSN_HIGH();

    return status;
}

void slNRF_Recive(uint8_t *buf, uint8_t len) {
    slNRF_ReadPayload(buf, len);

    //Clear the two possible interrupt flags with one command
    slNRF_SetRegister(STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));

    CSN_LOW();
    slSPI_TransferInt(FLUSH_RX);
    CSN_HIGH();
}

uint8_t slNRF_Available() {
    if (!(slNRF_GetRegister(FIFO_STATUS, 0) & _BV(RX_EMPTY))) {

        // // If the caller wants the pipe number, include that
        // if ( pipe_num ){
        //   uint8_t status = slSPI_TransferInt(0);
        //   *pipe_num = ( status >> RX_P_NO ) & 0x7;//0b111;
        // }
        return 1;
    }
    return 0;
}
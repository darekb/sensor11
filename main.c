#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define showDebugDataMain 1

#include "main.h"


#include "slUart.h"


#include "slNRF24.h"
#include "slBME180Measure.h"
#include "slI2C.h"
#include "BME280.h"
#include "slSPI.h"
#include "slAdc.h"

#define SENSOR_ID 11

void clearData();

void setupTimer();

void setupInt0();


void goSleep();

void goReset();

void compareStrings();

uint8_t setBME280Mode();

uint8_t getDataFromBME280();

void measuerADC();

uint8_t sendVianRF24L01();

uint8_t data[9];
uint8_t status;
struct MEASURE BME180measure = {0, 0, 0, 0, 0};
volatile uint8_t stage = 0;
volatile uint8_t inSleep = 0;
volatile uint16_t counter = 0;
const char startStringSensor11[] = {'s', 't', 'a', 'r', 't', '-', 's', '1', '1'};
uint8_t *arr;

int main(void) {
    slUART_SimpleTransmitInit();
    slUART_WriteString("start.\r\n");
    slI2C_Init();
    if (BME280_Init(BME280_OS_T_1, BME280_OS_P_1, BME280_OS_H_1, BME280_FILTER_OFF, BME280_MODE_FORCED,
                    BME280_TSB_1000)) {
        slUART_WriteString("BMP280 init error.\r\n");
    } else {
        slUART_WriteString("BMP280 init done.\r\n");
    }
    slSPI_Init();
    slNRF24_IoInit();
    setupTimer();
    setupInt0();
    slNRF24_Init();
    slADC_init();
    sei();
    stage = 10;

    while (1) {
        switch (stage) {
            case 9:
                goSleep();
                break;
            case 10:
                goReset();
            break;
            case 12:
                compareStrings();
                break;
            case 13:
                setBME280Mode();
                break;
            case 14:
                getDataFromBME280();
                break;
            case 15:
                measuerADC();
                break;
            case 16:
                sendVianRF24L01();
                break;
        }
    }
    return 0;
}


void clearData() {
    for (uint8_t i = 0; i < 9; i++) {
        data[i] = 0;
    };
}

void setupTimer() {
    TCCR1B |= (1 << CS12) | (1 << CS10);//prescaler 1024
    //TCNT1 = 0x0000;//Clear the timer counter register.
    TIMSK1 |= (1 << TOIE1);//przerwanie przy przepłnieniu timera0
    sei();
}

void setupInt0() {
    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    // PD2 (PCINT0 pin) is now an input
    PORTD |= (1 << PORTD2);    // turn On the Pull-up
    // PD2 is now an input with pull-up enabled
    EICRA |= (1 << ISC01);// INT0 falling edge PD2
    EICRA &= ~(1 << ISC00);// INT0 falling edge PD2
    EIMSK |= (1 << INT0);     // Turns on INT0
    sei();
}


//stage 9
void goSleep(){
    // slUART_WriteStringNl("Sensor11 sleep");
    // _delay_ms(10);
    inSleep = 1;
    sei();
    slNRF24_PowerDown();
    stage = 0;
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer2_disable();
    power_twi_disable();
    sleep_mode();
    sleep_disable();
    power_all_enable();
}
//stage 10
void goReset(){
    counter = 0;
    inSleep = 0;
    slUART_WriteStringNl("Sensor11 Reset");
    slNRF24_Reset();
    slNRF24_FlushTx();
    slNRF24_FlushRx();
    slNRF24_RxPowerUp();
    //wait for inerupt
    stage = 0;
}


//stage 12
void compareStrings() {
    counter = 0;
    inSleep = 0;
    slNRF24_GetRegister(R_RX_PAYLOAD,data,9);
    uint8_t go = 0;
    uint8_t i = sizeof(data);
    while (i--) {
        if (data[i] == startStringSensor11[i]) {
            go++;
        }
    }
    if (go == sizeof(data)) {
        //slUART_WriteStringNl("Sensor11 got start command");
        clearData();
        stage = 13;
    } else {
        goReset();
        stage = 0;//wait for interupt from got data
    }
}

//stage13
uint8_t setBME280Mode() {
    counter = 0;
    inSleep = 0;
    //slUART_WriteStringNl("Sensor11 reset BME280");
    if (BME280_SetMode(BME280_MODE_FORCED)) {
        slUART_WriteString("BME280 set forced mode error!\r\n");
        return 1;
    }
    stage = 14;
    return 0;
}

//stage 14
uint8_t getDataFromBME280() {
    counter = 0;
    inSleep = 0;
    float temperature, humidity, pressure;
    if (BME280_ReadAll(&temperature, &pressure, &humidity)) {

        slUART_WriteString("BME280 read error!\r\n");
        return 1;
    }
    BME180measure.temperature = calculateTemperature(temperature);
    BME180measure.humidity = calculateHumidity(humidity);
    BME180measure.pressure = calculatePressure(pressure);
    BME180measure.voltage = 323;
    BME180measure.sensorId = SENSOR_ID;
    //slUART_WriteStringNl("Sensor11 got data from BME280");
    // slUART_LogDecNl(BME180measure.temperature);
    // slUART_LogDecNl(BME180measure.humidity);
    // slUART_LogDecNl(BME180measure.pressure);
    stage = 15;
    return 0;
}

//stage 15
void measuerADC(){
    counter = 0;
    inSleep = 0;
    //slUART_WriteStringNl("Sensor11 measure ADC");
    float wynik = 0;
    for(uint8_t i = 0; i<12; i++){
        wynik = wynik + slADC_measure(PC1);
    }
    wynik = (((110*((wynik/12)*100))/102300)*350)/110;
    BME180measure.voltage = (uint16_t)wynik;
    stage = 16;
}

//stage16
uint8_t sendVianRF24L01() {
    counter = 0;
    inSleep = 0;
    slNRF24_FlushTx();
    slNRF24_FlushRx();
    slNRF24_Reset();
    fillBuferFromMEASURE(BME180measure, data);
    slUART_WriteStringNl("Sensor11 Sending data");
    slNRF24_TxPowerUp();
    slNRF24_TransmitPayload(&data, 9);
    clearData();
    stage = 9;//goSleep
    return 0;
}

ISR(TIMER1_OVF_vect) {
    //co 4.09sek.
    if(inSleep == 1){
        counter = counter + 1;
        if (counter == 6) {//24.00672 sek Next mesurements
            counter = 0;
            stage = 10;
        } else {
            stage = 9;
        }
    }
}

ISR(INT0_vect) {
    status = 0;
    counter = 0;
    slNRF24_GetRegister(STATUS, &status, 1);
    //slUART_LogBinaryNl(status);
    cli();
    if ((status & (1 << 6)) != 0) {
        stage = 12;//CompareStrings
    }
    if ((status & (1 << 5)) != 0) {
        stage = 9;//goSleep
    }
    sei();
}

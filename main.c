#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

#define LED (1 << PB0)
#define LED_TOG PORTB ^= LED
#define SENSOR_ID 11

void clearData();
void setupTimer();

void nrf24_Start();

//sensor11
void waitForStart();

uint8_t setBME280Mode();

uint8_t getDataFromBME280();

uint8_t sendVianRF24L01();

uint8_t pipe1[] = {0xF0, 0xF0, 0xF0, 0xF0, 0xE1};
uint8_t pipe2[] = {0xF0, 0xF0, 0xF0, 0xF0, 0xD2};
uint8_t data[9];
float t = 0;
struct MEASURE BME180measure = {0, 0, 0, 0, 0};
volatile uint8_t stage;
volatile uint16_t counter = 0;
uint8_t t1[9] = {0x73, 0x74, 0x61, 0x72, 0x74, 0x2d, 0x73, 0x31, 0x31};

const char startStringSensor11[] = {'s','t','a','r','t','-','s','1','1'};
uint8_t i =0;
int main(void) {
    slUART_SimpleTransmitInit();
    slI2C_Init();
    setupTimer();
    nrf24_Start();
    if (BME280_Init(BME280_OS_T_1, BME280_OS_P_1, BME280_OS_H_1, BME280_FILTER_OFF, BME280_MODE_FORCED,
                  BME280_TSB_1000)) {
#if showDebugDataMain == 1
        slUART_WriteString("BMP280 init error.\r\n");
#endif
    } else {
#if showDebugDataMain == 1
        slUART_WriteString("BMP280 init done.\r\n");
#endif
    }
    stage = 11;
    while (1) {
        switch (stage) {
            case 11:
                waitForStart();
                break;
            case 12:
                setBME280Mode();
                break;
            case 13:
                getDataFromBME280();
                break;
            case 14:
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

void setupTimer(){
    TCCR0B |= (1 << CS02) | (1 << CS00);//prescaler 1024
    TIMSK0 |= (1 << TOIE0);//przerwanie przy przepłnieniu timera0
    sei();
}

void nrf24_Start() {
    slNRF_Init();
    slNRF_OpenWritingPipe(pipe2, 9);
    slNRF_OpenReadingPipe(pipe1, 9, 1);
    slNRF_SetDataRate(RF24_2MBPS);
    slNRF_SetPALevel(RF24_PA_MAX);
    slNRF_SetChannel(100);
    slNRF_EnableDynamicPayloads();
    slNRF_EnableAckPayload();
    slNRF_SetRetries(0, 15);
    slNRF_AutoAck(1);
    //slNRF_showDebugData();
    slNRF_PowerUp();
    slNRF_StartListening();
    clearData();
}



//sensor11

//stage 11
void waitForStart() {
    counter = 0;
    if (slNRF_Available()) {
        clearData();
        slNRF_Recive(data, sizeof(data));
        if (strcmp((char *) data, startStringSensor11)) {
            slUART_WriteStringNl("got start command");
            stage = 12;
            _delay_ms(1000);
        }
        slNRF_FlushTX();
        slNRF_FlushRX();
    }

}

//stage 12
uint8_t setBME280Mode() {
    counter = 0;
    slUART_WriteStringNl("reset BME280");
    if (BME280_SetMode(BME280_MODE_FORCED)) {
#if showDebugDataMain == 1
        slUART_WriteString("Sensor set forced mode error!\r\n");
#endif
        return 1;
    }
    stage = 13;
    return 0;
}

//stage 13
uint8_t getDataFromBME280() {
    counter = 0;
    float temperature, humidity, pressure;
    if (BME280_ReadAll(&temperature, &pressure, &humidity)) {
#if showDebugDataMain == 1
        slUART_WriteString("Sensor read error!\r\n");
#endif
        return 1;
    }
    BME180measure.temperature = calculateTemperature(temperature);
    BME180measure.humidity = calculateHumidity(humidity);
    BME180measure.pressure = calculatePressure(pressure);
    BME180measure.voltage = 323;
    BME180measure.sensorId = SENSOR_ID;
    slUART_LogDecNl(BME180measure.temperature);
    slUART_LogDecNl(BME180measure.humidity);
    slUART_LogDecNl(BME180measure.pressure);
    slUART_WriteStringNl("got data from BME280");
    slNRF_FlushTX();
    slNRF_FlushRX();
    stage = 14;
    return 0;
}

//stage 14
uint8_t sendVianRF24L01() {
    counter = 0;
    slUART_WriteStringNl("Sending data to server");
    //LED_TOG;
    fillBuferFromMEASURE(BME180measure, data);
    slNRF_StopListening();
    if (!slNRF_Sent(data, sizeof(data))) {
        slUART_WriteStringNl("Fail\n");
    } else {
        slUART_WriteStringNl("SendOk\n");
    }
    clearData();
    slNRF_StartListening();
    //LED_TOG;
    stage = 11;
    return 0;
}


ISR(TIMER0_OVF_vect) {
  //co 0.01632sek.
  counter = counter + 1;
  if (counter == 1840) {//15.014400000000002*2 sek
    counter = 0;
    //if(stage == 0){
      stage = 11;
    //}
  }
}
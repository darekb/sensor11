#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>


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

#define SENSOR_ID 11

void clearData();
void setupTimer();
void nrf24_Start();


void goSleep();
void wakeUp();
void waitForStart();
void compareStrings();
uint8_t setBME280Mode();
uint8_t getDataFromBME280();
uint8_t sendVianRF24L01();

uint8_t pipe1[] = {0xF0, 0xF0, 0xF0, 0xF0, 0xE1};
uint8_t pipe2[] = {0xF0, 0xF0, 0xF0, 0xF0, 0x95};
uint8_t data[9];
struct MEASURE BME180measure = {0, 0, 0, 0, 0};
volatile uint8_t stage;
volatile uint16_t counter = 0;
volatile int f_timer = 0;
const char startStringSensor11[] = {'s', 't', 'a', 'r', 't', '-', 's', '1', '1'};

int main(void) {

    slUART_SimpleTransmitInit();
    slUART_WriteString("start.\r\n");
    slI2C_Init();
    setupTimer();
    nrf24_Start();
    if (BME280_Init(BME280_OS_T_1, BME280_OS_P_1, BME280_OS_H_1, BME280_FILTER_OFF, BME280_MODE_FORCED,
                    BME280_TSB_1000)) {
        slUART_WriteString("BMP280 init error.\r\n");
    } else {
        slUART_WriteString("BMP280 init done.\r\n");
    }
    stage = 11;
    while (1) {
        switch (stage) {
            case 9://sleep
                goSleep();
                break;
            case 10:
                wakeUp();
                break;
            case 11:
                waitForStart();
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
    //TCCR1A = 0x00;/* Normal timer operation.*/
    /* Clear the timer counter register.
   * You can pre-load this register with a value in order to 
   * reduce the timeout period, say if you wanted to wake up
   * ever 4.0 seconds exactly.
   */
    //TCNT1 = 0x0000;

    TCCR1B |= (1 << CS12);//prescaler = 256
    //TCNT1 = 0;// initialize counter
    TIMSK1 |= (1 << TOIE1);//enable overflow interupt

    // TCCR0B |= (1 << CS02) | (1 << CS00);//prescaler 1024
    // TIMSK0 |= (1 << TOIE0);//przerwanie przy przepłnieniu timera0
    sei();
}

void nrf24_Start() {
    slNRF_Init();
    slNRF_FlushTX();
    slNRF_OpenWritingPipe(pipe2, 9);
    slNRF_OpenReadingPipe(pipe1, 9, 1);
    slNRF_SetDataRate(RF24_250KBPS);
    slNRF_SetPALevel(RF24_PA_MAX);
    slNRF_SetChannel(77);
    slNRF_DisableDynamicPayloads();
    slNRF_EnableAckPayload();
    slNRF_SetRetries(0, 3);
    slNRF_AutoAck(1);
    slNRF_PowerUp();
    slNRF_StartListening();
    clearData();
}
//stage 9
void goSleep() {
    counter = counter + 1;
    if (counter >= 20) {//15.728399999999999 sek
        f_timer = 0;
        stage = 10;
        counter = 0;
        /* Re-enable the peripherals. */
        power_all_enable();
    } else {
        f_timer = 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        /* Disable all of the unused peripherals. This will reduce power
         * consumption further and, more importantly, some of these
         * peripherals may generate interrupts that will wake our Arduino from
         * sleep!
         */
        power_adc_disable();
        power_spi_disable();
        power_timer0_disable();
        power_timer2_disable();
        power_twi_disable();
        /* Now enter sleep mode. */
        sleep_mode();
        /* The program will continue from here after the timer timeout*/
        sleep_disable(); /* First thing to do is disable sleep. */
    }
}

//stage 10

void wakeUp() {
    slUART_WriteStringNl("wakeUp");
    counter = 0;
    slNRF_FlushTX();
    slNRF_FlushRX();
    slNRF_PowerUp();
    slNRF_StartListening();
    clearData();
    stage = 11;
}

//stage 11
void waitForStart() {
    counter = 0;
    if (slNRF_Available()) {
        slNRF_Recive(data, sizeof(data));
        slUART_WriteStringNl("got data from server");
        slUART_WriteStringNl((char *) data);
        slNRF_StopListening();
        slNRF_PowerDown();
        stage = 12;
    }
}

//stage 12
void compareStrings() {
    uint8_t go = 0;
    uint8_t i = sizeof(data);
    while (i--) {
        if (data[i] == startStringSensor11[i]) {
            go++;
        }
    }
    //slUART_LogDecNl(go);
    if (go == sizeof(data)) {
        slUART_WriteStringNl("got start command");
        stage = 13;
        clearData();
    } else {
        stage = 11;
    }
}

//stage13
uint8_t setBME280Mode() {
    counter = 0;
    slUART_WriteStringNl("reset BME280");
    if (BME280_SetMode(BME280_MODE_FORCED)) {
        slUART_WriteString("Sensor set forced mode error!\r\n");
        return 1;
    }
    stage = 14;
    return 0;
}

//stage 14
uint8_t getDataFromBME280() {
    counter = 0;
    float temperature, humidity, pressure;
    if (BME280_ReadAll(&temperature, &pressure, &humidity)) {

        slUART_WriteString("Sensor read error!\r\n");
        return 1;
    }
    BME180measure.temperature = calculateTemperature(temperature);
    BME180measure.humidity = calculateHumidity(humidity);
    BME180measure.pressure = calculatePressure(pressure);
    BME180measure.voltage = 323;
    BME180measure.sensorId = SENSOR_ID;
    slUART_WriteStringNl("got data from BME280");
    slUART_LogDecNl(BME180measure.temperature);
    slUART_LogDecNl(BME180measure.humidity);
    slUART_LogDecNl(BME180measure.pressure);
    stage = 15;
    return 0;
}

//stage15
uint8_t sendVianRF24L01() {
    counter = 0;
    _delay_ms(4500);
    fillBuferFromMEASURE(BME180measure, data);
    slUART_WriteStringNl("Sending data to server");
    slNRF_PowerUp();
    if (!slNRF_Sent(data, sizeof(data))) {
        clearData();
        slNRF_PowerDown();
        counter = 0;
        stage = 0;
        slUART_WriteStringNl("Fail\n");
        stage = 9;//go sleep
        return 1;
    } else {
        slUART_WriteStringNl("SendOk\n");
        slNRF_PowerDown();
        counter = 0;
        stage = 0;
        clearData();
        stage = 9;//go sleep
    }
    return 0;
}

ISR(TIMER1_OVF_vect) {
    //co 1.04856 sek.
    if(f_timer == 1){
        stage = 9;
    }

}
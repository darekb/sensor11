//
// Created by dariusz on 25.02.17.
//
#include <avr/io.h>
#include <string.h>
#include "slBME180Measure.h"
//#include "slUart.h"

void fillBuferFromMEASURE(const struct MEASURE structure, uint8_t *buffer) {
    memcpy(buffer, &structure, sizeof(struct MEASURE));
}

struct MEASURE returnMEASUREFromBuffer(uint8_t *buffer) {
    struct MEASURE tmp;
    memcpy(&tmp, buffer, 9 * sizeof(uint8_t));
    return tmp;
}

int16_t calculateTemperature(float temperature) {
    return (int16_t) ((float) temperature * (int16_t) 100);
}

uint16_t calculateHumidity(float humidity) {
    return (uint16_t) ((float) humidity * (uint16_t) 100);
}

int16_t calculatePressure(float pressure) {
    return (int16_t) ((float) pressure - (float) 100000);
}

uint16_t calculateVoltage(float voltage) {
    return (uint16_t) ((float) voltage * (uint16_t) 100);
}

//void testBME180Measure(void) {
//    struct MEASURE BME180measure = {856, 6818, -1235, 33, 11};
//    struct MEASURE t2 = {0, 0, 0, 0, 11};
//    uint8_t buffer[8];
//    fillBuferFromMEASURE(BME180measure, buffer);
//    slUART_WriteString("\r\n***************************\r\n");
//    slUART_WriteBuffer(buffer, 8);
//    slUART_WriteString("\r\n***************************\r\n");
//    float t = -12.455467;
//    BME180measure.temperature = calculateTemperature(t);
//    t = 71.567;
//    BME180measure.humidity = calculateHumidity(t);
//    t = 98856.456;
//    BME180measure.pressure = calculatePressure(t);
//    t2 = returnMEASUREFromBuffer(buffer);
//    slUART_LogDecNl(t2.temperature);
//    slUART_LogDecNl(BME180measure.temperature);
//    slUART_LogDecNl(t2.humidity);
//    slUART_LogDecNl(BME180measure.humidity);
//    slUART_LogDecNl(t2.pressure);
//    slUART_LogDecNl(BME180measure.pressure);
//}
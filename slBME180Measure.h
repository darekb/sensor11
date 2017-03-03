//
// Created by dariusz on 25.02.17.
//

#ifndef CMAKE_AVR_SLBME180MEASURE_H
#define CMAKE_AVR_SLBME180MEASURE_H


struct MEASURE {
   int16_t temperature;
   uint16_t humidity;
   int16_t pressure;//(pressure - 1000)*100
   uint16_t voltage;
   uint8_t sensorId;
};

void fillBuferFromMEASURE (const struct MEASURE structure, uint8_t *buffer);
struct MEASURE returnMEASUREFromBuffer (uint8_t *buffer);

int16_t calculateTemperature(float temperature);
uint16_t calculateHumidity(float humidity);
int16_t calculatePressure(float pressure);
uint16_t calculateVoltage(float voltage);
//void testBME180Measure(void);

#endif //CMAKE_AVR_SLBME180MEASURE_H

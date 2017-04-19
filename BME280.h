/*********************************************************************************
Copyright (c) 2016, Cosmin Plasoianu
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 ********************************************************************************/

#ifndef _BME280_H_
#define _BME280_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define showDebugDataBME280 0


#define ID_REG            0xD0
#define PRESS_MSB_REG     0xF7
#define CALIB_00_REG      0x88
#define CALIB_26_REG      0xE1
#define CONFIG_REG        0xF5
#define CTRL_MEAS_REG     0xF4
#define STATUS_REG        0xF3
#define CTRL_HUM_REG      0xF2
#define TEMP_DIG_ADDR  	  0x88
#define PRESS_DIG_ADDR    0x8E
#define HUM_DIG_ADDR1     0xA1
#define HUM_DIG_ADDR2     0xE1

//Device Address
#define BME280_I2C_ADDR   0xEC  //0xEC or 0xEE

//Configuration parameters
//Temperature oversampling
#define BME280_OS_T_SKP   0x00
#define BME280_OS_T_1     0x01
#define BME280_OS_T_2     0x02
#define BME280_OS_T_4     0x03
#define BME280_OS_T_8     0x04
#define BME280_OS_T_16    0x05

//Pressure oversampling
#define BME280_OS_P_SKP   0x00
#define BME280_OS_P_1     0x01
#define BME280_OS_P_2     0x02
#define BME280_OS_P_4     0x03
#define BME280_OS_P_8     0x04
#define BME280_OS_P_16    0x05

//Humidity oversampling
#define BME280_OS_H_SKP   0x00
#define BME280_OS_H_1     0x01
#define BME280_OS_H_2     0x02
#define BME280_OS_H_4     0x03
#define BME280_OS_H_8     0x04
#define BME280_OS_H_16    0x05

//Filter coefficient
#define BME280_FILTER_OFF 0x00
#define BME280_FILTER_2   0x01
#define BME280_FILTER_4   0x02
#define BME280_FILTER_8   0x03
#define BME280_FILTER_16  0x04

//Mode
#define BME280_MODE_SLEEP  0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03

//Standby time - ms
#define BME280_TSB_05      0x00  // 0.5
#define BME280_TSB_62      0x01  //62.5
#define BME280_TSB_125     0x02
#define BME280_TSB_250     0x03
#define BME280_TSB_500     0x04
#define BME280_TSB_1000    0x05
#define BME280_TSB_10      0x06
#define BME280_TSB_20      0x07


//User API
uint8_t BME280_Init(uint8_t os_t, uint8_t os_p, uint8_t os_h, uint8_t filter, uint8_t mode, uint8_t t_sb);
uint8_t BME280_ReadAll(float *t, float *p, float *h);
uint8_t BME280_SetMode(uint8_t mode);

#endif /* _BME280_H_ */

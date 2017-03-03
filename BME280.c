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

#include <avr/io.h>
#include "slI2C.h"
#include "BME280.h"
#if showDebugDataBME280 == 1
#include <string.h>
#include <stdio.h>
#include "slUart.h"
#endif


static struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} CalibParam;

int32_t t_fine;        // t_fine carries fine temperature as global value


uint8_t I2C_WriteData(uint8_t device_addr, uint8_t register_addr, uint8_t *data, uint8_t length) {
  uint8_t cnt = 0;
  if (slI2C_Start()) {
    return 1;
  }
  device_addr &= 0xFE;//Device address
  if (slI2C_SetSLA(device_addr)) {
    return 1;
  }
  if (slI2C_WriteByte(register_addr)) {
    return 1;
  }
  while (cnt != length) {
    if (slI2C_WriteByte(data[cnt++])) {
      return 1;
    }
  }
  if (slI2C_Stop()) {
    return 1;
  }
  return 0;
}

uint8_t I2C_ReadData(uint8_t device_addr, uint8_t register_addr, uint8_t *data, uint8_t length) {
  uint8_t cnt;
  if (slI2C_Start()) {
    return 1;
  }
  device_addr &= 0xFE;//Device address
  if (slI2C_SetSLA(device_addr)) {
    return 1;
  }
  if (slI2C_WriteByte(register_addr)) {
    return 1;
  }
  if (slI2C_Start()) {
    return 1;
  }
  device_addr |= 0x01;
  if (slI2C_SetSLA(device_addr)) {
    return 1;
  }
  if (length) {
    if (length > 1) {
      for (cnt = 0; cnt < length - 1; cnt++) {
        if (slI2C_ReadByte_ACK()) {
          return 1;
        } else {
          data[cnt] = slI2C_returnReadValue();
        }
      }
    }
    if (slI2C_ReadByte_NACK()) {
      return 1;
    } else {
      data[length - 1] = slI2C_returnReadValue();
    }
    if (slI2C_Stop()) {
      return 1;
    }              //Stop
    return 0;
  }
  return 0;
}


uint8_t BME280_getCalibrationData() {

  uint8_t Buff[18];
  uint8_t dig[32];
  uint8_t cnt, ord;
  ord = 0;
  // Temp. Dig
  if (I2C_ReadData(BME280_I2C_ADDR, TEMP_DIG_ADDR, Buff, 6)) {
#if showDebugDataBME280 == 1
    for (cnt = 0; cnt < 6; cnt++) {
        slUART_WriteString("BME280_Init - Tmp[");
        slUART_LogDec(cnt);
        slUART_WriteString("]: ");
        slUART_LogBinary(dig[cnt]);
    }
#endif
    return 1;
  }
  for (cnt = 0; cnt < 6; cnt++) {
    dig[ord++] = Buff[cnt];
  }
  CalibParam.dig_T1 = (dig[1] << 8) | dig[0];
  CalibParam.dig_T2 = (dig[3] << 8) | dig[2];
  CalibParam.dig_T3 = (dig[5] << 8) | dig[4];

// Pressure Dig
  if (I2C_ReadData(BME280_I2C_ADDR, PRESS_DIG_ADDR, Buff, 18)) {
#if showDebugDataBME280 == 1
    for (cnt = 0; cnt < 17; cnt++) {
        slUART_WriteString("BME280_Init - Press[");
        slUART_LogDec(cnt);
        slUART_WriteString("]: ");
        slUART_LogBinary(Buff[cnt]);
    }
#endif
    return 1;
  }
  for (cnt = 0; cnt < 18; cnt++) {
    dig[ord++] = Buff[cnt];
  }
  CalibParam.dig_P1 = (dig[7] << 8) | dig[6];
  CalibParam.dig_P2 = (dig[9] << 8) | dig[8];
  CalibParam.dig_P3 = (dig[11] << 8) | dig[10];
  CalibParam.dig_P4 = (dig[13] << 8) | dig[12];
  CalibParam.dig_P5 = (dig[15] << 8) | dig[14];
  CalibParam.dig_P6 = (dig[17] << 8) | dig[16];
  CalibParam.dig_P7 = (dig[19] << 8) | dig[18];
  CalibParam.dig_P8 = (dig[21] << 8) | dig[20];
  CalibParam.dig_P9 = (dig[23] << 8) | dig[22];


  // Humidity Dig 1
  if (I2C_ReadData(BME280_I2C_ADDR, HUM_DIG_ADDR1, Buff, 1)) {
#if showDebugDataBME280 == 1
    slUART_WriteString("BME280_Init - Hum1[");
    slUART_LogDec(0);
    slUART_WriteString("]: ");
    slUART_LogBinary(Buff[0]);
#endif
    return 1;
  }
  dig[ord++] = Buff[0];

// Humidity Dig 2
  if (I2C_ReadData(BME280_I2C_ADDR, HUM_DIG_ADDR2, Buff, 7)) {
#if showDebugDataBME280 == 1
    for (cnt = 0; cnt < 6; cnt++) {
        slUART_WriteString("BME280_Init - Buff[");
        slUART_LogDec(cnt);
        slUART_WriteString("]: ");
        slUART_LogBinary(Buff[cnt]);
    }
#endif
    return 1;
  }
  for (cnt = 0; cnt < 7; cnt++) {
    dig[ord++] = Buff[cnt];
  }

  CalibParam.dig_H1 = dig[24];
  CalibParam.dig_H2 = (dig[26] << 8) | dig[25];
  CalibParam.dig_H3 = dig[27];
  CalibParam.dig_H4 = (dig[28] << 4) | (dig[29] & 0x0F);
  CalibParam.dig_H5 = (dig[30] << 4) | ((dig[29] >> 4) & 0x0F);
  CalibParam.dig_H6 = dig[31];

#if showDebugDataBME280 == 1

  slUART_WriteString("CalibParam.dig_T1: ");
  slUART_LogBinary(CalibParam.dig_T1);
  slUART_WriteString("CalibParam.dig_T2: ");
  slUART_LogBinary(CalibParam.dig_T2);
  slUART_WriteString("CalibParam.dig_T3: ");
  slUART_LogBinary(CalibParam.dig_T3);

  slUART_WriteString("CalibParam.dig_P1: ");
  slUART_LogBinary(CalibParam.dig_P1);
  slUART_WriteString("CalibParam.dig_P2: ");
  slUART_LogBinary(CalibParam.dig_P2);
  slUART_WriteString("CalibParam.dig_P3: ");
  slUART_LogBinary(CalibParam.dig_P3);
  slUART_WriteString("CalibParam.dig_P4: ");
  slUART_LogBinary(CalibParam.dig_P4);
  slUART_WriteString("CalibParam.dig_P5: ");
  slUART_LogBinary(CalibParam.dig_P5);
  slUART_WriteString("CalibParam.dig_P6: ");
  slUART_LogBinary(CalibParam.dig_P6);
  slUART_WriteString("CalibParam.dig_P7: ");
  slUART_LogBinary(CalibParam.dig_P7);
  slUART_WriteString("CalibParam.dig_P8: ");
  slUART_LogBinary(CalibParam.dig_P8);
  slUART_WriteString("CalibParam.dig_P9: ");
  slUART_LogBinary(CalibParam.dig_P9);


  slUART_WriteString("CalibParam.dig_H1: ");
  slUART_LogBinary(CalibParam.dig_H1);
  slUART_WriteString("CalibParam.dig_H2: ");
  slUART_LogBinary(CalibParam.dig_H2);
  slUART_WriteString("CalibParam.dig_H3: ");
  slUART_LogBinary(CalibParam.dig_H3);
  slUART_WriteString("CalibParam.dig_H4: ");
  slUART_LogBinary(CalibParam.dig_H4);
  slUART_WriteString("CalibParam.dig_H5: ");
  slUART_LogBinary(CalibParam.dig_H5);
  slUART_WriteString("CalibParam.dig_H6: ");
  slUART_LogBinary(CalibParam.dig_H6);
#endif
  return 0;
}


/**********************************************************************
Return: 0 	  - Everything OK
		non 0 - Failed
Parameters:	os_t - Temperature Oversampling
			os_p - Pressure Oversampling
			os_h - Humidity Oversampling
			filter - Filter coefficient
			mode - Mode (Sleep/Forced/Normal)
			t_sb - Standby time between conversions
**********************************************************************/
uint8_t BME280_Init(uint8_t os_t, uint8_t os_p, uint8_t os_h,
                    uint8_t filter, uint8_t mode, uint8_t t_sb) {
  uint8_t ID = 0;
  uint8_t Temp;

  if (I2C_ReadData(BME280_I2C_ADDR, ID_REG, &ID, 1)) {
#if showDebugDataBME280 == 1
    slUART_WriteString("ID:  ");
    slUART_LogBinary(ID);
#endif
    return 1;
  }
  if (ID != 0x60)
    return 1;
  if(BME280_getCalibrationData()){
    return 1;
  }

  Temp = (t_sb << 5) | ((filter & 0x07) << 2);                    //config (0xB4)
  if (I2C_WriteData(BME280_I2C_ADDR, CONFIG_REG, &Temp, 1)) {
    return 1;
  }

  Temp = os_h & 0x07;                                                //hum (0x05)
  if (I2C_WriteData(BME280_I2C_ADDR, CTRL_HUM_REG, &Temp, 1)) {
    return 1;
  }

  Temp = (os_t << 5) | ((os_p & 0x07) << 2) | (mode & 0x03);        //meas (0xB7)
  if (I2C_WriteData(BME280_I2C_ADDR, CTRL_MEAS_REG, &Temp, 1)) {
    return 1;
  }

  return 0;
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of �5123� equals 51.23 DegC.
float BME280_CompensateT(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t) CalibParam.dig_T1 << 1))) * ((int32_t) CalibParam.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t) CalibParam.dig_T1)) * ((adc_T >> 4) - ((int32_t) CalibParam.dig_T1))) >> 12) *
          ((int32_t) CalibParam.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
#if showDebugDataBME280 == 1
  slUART_WriteString("BME280_CompensateT: ");
  slUART_LogBinary((uint8_t) (T & 0xFF));
  slUART_LogBinary((uint8_t) ((T >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((T >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((T >> 24)));
#endif
  return T / 100.0;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of �24674867� represents 24674867/256 = 96386.2 Pa = 963.862 hPa

int64_t BME280_CompensateP(int32_t adc_P) {
  int64_t var1, var2, p;
  int64_t final;
  var1 = (int64_t) t_fine - 128000;
  var2 = var1 * var1 * (int64_t) CalibParam.dig_P6;
  var2 = var2 + ((var1 * (int64_t) CalibParam.dig_P5) << 17);
  var2 = var2 + (((int64_t) CalibParam.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t) CalibParam.dig_P3) >> 8) + ((var1 * (int64_t) CalibParam.dig_P2) << 12);
  var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) CalibParam.dig_P1) >> 33;
  if (var1 == 0) { return 0; }                                      // Don't divide by zero.
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t) CalibParam.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t) CalibParam.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t) CalibParam.dig_P7) << 4);

  final = ((uint32_t) p) / 256.0;

#if showDebugDataBME280 == 1
  slUART_WriteString("BME280_CompensateP: ");
  slUART_LogBinary((uint8_t) (p & 0xFF));
  slUART_LogBinary((uint8_t) ((p >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((p >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((p >> 24)));
#endif
  return final;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of �47445� represents 47445/1024 = 46.333 %RH
float BME280_CompensateH(int32_t adc_H) {
  int32_t v_x1_u32;
  v_x1_u32 = (t_fine - ((int32_t) 76800));
  v_x1_u32 = (
      ((((adc_H << 14) - (((int32_t) CalibParam.dig_H4) << 20) - (((int32_t) CalibParam.dig_H5) * v_x1_u32)) +
        ((int32_t) 16384)) >> 15) * (((((((v_x1_u32 * ((int32_t) CalibParam.dig_H6)) >> 10) *
                                         (((v_x1_u32 * ((int32_t) CalibParam.dig_H3)) >> 11) +
                                          ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                      ((int32_t) CalibParam.dig_H2) + 8192) >> 14));
  v_x1_u32 = (v_x1_u32 - (((((v_x1_u32 >> 15) * (v_x1_u32 >> 15)) >> 7) * ((int32_t) CalibParam.dig_H1)) >> 4));
  v_x1_u32 = (v_x1_u32 < 0 ? 0 : v_x1_u32);
  v_x1_u32 = (v_x1_u32 > 419430400 ? 419430400 : v_x1_u32);
  v_x1_u32 = (v_x1_u32 >> 12);
#if showDebugDataBME280 == 1
  slUART_WriteString("BME280_CompensateH: ");
  slUART_LogBinary((uint8_t) (v_x1_u32 & 0xFF));
  slUART_LogBinary((uint8_t) ((v_x1_u32 >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((v_x1_u32 >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((v_x1_u32 >> 24)));
#endif
  return v_x1_u32 / 1024.0;
}

/**********************************************************************
Return: 0 	  - Everything OK
		non 0 - Failed
Parameters:	t - Pointer to variable in which to write the temperature
			p - Pointer to variable in which to write the pressure
			h - Pointer to variable in which to write the humidity
**********************************************************************/

uint8_t BME280_ReadAll(float *t, float *p, float *h) {
  uint8_t Buff[8] = {0};
  uint32_t UncT, UncP, UncH;
#if showDebugDataBME280 == 1
  uint8_t cnt;
#endif
  if(BME280_getCalibrationData()){
    return 1;
  }

  if (I2C_ReadData(BME280_I2C_ADDR, PRESS_MSB_REG, Buff, 8)) {
#if showDebugDataBME280 == 1
    for (cnt = 0; cnt < 7; cnt++) {
        slUART_WriteString("Buff[");
        slUART_LogDec(cnt);
        slUART_WriteString("]: ");
        slUART_LogBinary(Buff[cnt]);
    }
#endif
    return 1;
  }

  UncP = (int32_t) (((uint32_t) Buff[0] << 12) | ((uint32_t) Buff[1] << 4) | ((uint32_t) Buff[2] >> 4));

  UncT = (int32_t) (((uint32_t) Buff[3] << 12) | ((uint32_t) Buff[4] << 4) | ((uint32_t) Buff[5] >> 4));

  UncH = (int32_t) (((uint32_t) Buff[6] << 8) | (uint32_t) Buff[7]);

#if showDebugDataBME280 == 1
  slUART_WriteString("UncP: ");
  slUART_LogBinary((uint8_t) (UncP & 0xFF));
  slUART_LogBinary((uint8_t) ((UncP >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncP >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncP >> 24)));
  slUART_WriteString("UncT: ");
  slUART_LogBinary((uint8_t) (UncT & 0xFF));
  slUART_LogBinary((uint8_t) ((UncT >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncT >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncT >> 24)));
  slUART_LogBinary(UncT);
  slUART_WriteString("UncH: ");
  slUART_LogBinary((uint8_t) (UncH & 0xFF));
  slUART_LogBinary((uint8_t) ((UncH >> 8) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncH >> 16) & 0xFF));
  slUART_LogBinary((uint8_t) ((UncH >> 24)));
#endif

  *t = BME280_CompensateT(UncT);
  *p = BME280_CompensateP(UncP);
  *h = BME280_CompensateH(UncH);

  return 0;
}

/**********************************************************************
Return: 0 	  - Everything OK
		non 0 - Failed
Parameters:	mode - Mode (Sleep/Forced/Normal)
**********************************************************************/
uint8_t BME280_SetMode(uint8_t mode) {
  uint8_t RegVal = 0;

  mode &= 0x03;
  if (I2C_ReadData(BME280_I2C_ADDR, CTRL_MEAS_REG, &RegVal, 1)) {
    return 1;
  }
  RegVal &= 0xFC;
  RegVal |= mode;
  return I2C_WriteData(BME280_I2C_ADDR, CTRL_MEAS_REG, &RegVal, 1);
}
/*
 * slI2C.c
 *
 *  Created on: 24-05-2016
 *      Author: db
 */

#include <avr/io.h>
#include <util/twi.h>
#include "slI2C.h"

#if showDebugDataI2C == 1
#include "slUart.h"
#endif

uint8_t slI2C_readByteValue = 0;

uint8_t slI2C_statusFlags = 0;

uint8_t I2C_Error = 0;

static inline void slI2C_WaitForComplete() {
  while (!(TWCR & (1 << TWINT)));
}

static inline void slI2C_WaitTillStopWasSent() {
  while (TWCR & (1 << TWSTO));
}

static inline void slI2C_SetError(uint8_t err) {
  I2C_Error = err;
}

uint8_t slI2C_returnReadValue() {
  return slI2C_readByteValue;
}

#if showDebugDataI2C == 1

void showErrors(const char buff[]) {
    if (I2C_Error > 0) {
        slUART_WriteString(buff);
        slUART_WriteString(" - ");
        if (I2C_Error == 1) {
            slUART_WriteString("START Error \n");
        }
        if (I2C_Error == 3) {
            slUART_WriteString("no NACK Error \n");
        }
        if (I2C_Error == 4) {
            slUART_WriteString("no ACK Error \n");
        }
    }
}

#endif


uint8_t slI2C_SetSLA(uint8_t byte) {
  uint8_t twst;
  slI2C_WaitForComplete();
  I2C_Error = 0;
  TWDR = byte;
  TWCR = (1 << TWINT) | (1 << TWEN);
  slI2C_WaitForComplete();
  twst = TW_STATUS & 0xF8;
  if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) {
    slI2C_SetError(slI2C_NoACKError);
  }
#if showDebugDataI2C == 1
  showErrors("slI2C_SetSLA");
  if (I2C_Error) {
      slUART_WriteString("byte:  ");
      slUART_LogBinary(byte);
      slUART_WriteString("TW_STATUS:  ");
      slUART_LogBinary(TW_STATUS);
  }
#endif
  return I2C_Error;
}

void slI2C_Init() {
  slI2C_statusFlags |= (1 << slI2C_Init_Called);
  TWCR = (1 << TWEA) | (1 << TWEN);
  //set I2C bus speed
  TWBR = __TWBR;
}

uint8_t slI2C_Start() {
  I2C_Error = 0;
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  slI2C_WaitForComplete();
  if ((TW_STATUS != TW_START) && (TW_STATUS != TW_REP_START)) {
    slI2C_SetError(slI2C_STARTError);
  }
#if showDebugDataI2C == 1
  showErrors("slI2C_Start");
  if (I2C_Error) {
      slUART_WriteString("TW_STATUS:  ");
      slUART_LogBinary(TW_STATUS);
  }
#endif
  return I2C_Error;
}

uint8_t slI2C_Stop() {
  I2C_Error = 0;
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  slI2C_WaitTillStopWasSent();
#if showDebugDataI2C == 1
  showErrors("slI2C_Stop");
#endif
  return I2C_Error;
}

uint8_t slI2C_WriteByte(uint8_t byte) {
  uint8_t twst;
  slI2C_WaitForComplete();
  I2C_Error = 0;
  TWDR = byte;
  TWCR = (1 << TWINT) | (1 << TWEN);
  slI2C_WaitForComplete();
  twst = TW_STATUS & 0xF8;
  if ((twst != TW_MT_DATA_ACK) && (twst != TW_MR_DATA_ACK)) {
    slI2C_SetError(slI2C_NoACKError);
  }
#if showDebugDataI2C == 1
  showErrors("slI2C_WriteByte");
  if (I2C_Error) {
      slUART_WriteString("byte:  ");
      slUART_LogBinary(byte);
      slUART_WriteString("TW_STATUS:  ");
      slUART_LogBinary(TW_STATUS);
  }
#endif
  return I2C_Error;
}

#if compileREadFunctions == 1

uint8_t slI2C_ReadByte_NACK() {
  I2C_Error = 0;
  TWCR = (1 << TWINT) | (1 << TWEN);
  slI2C_WaitForComplete();
  if (TW_STATUS != TW_MR_DATA_NACK) slI2C_SetError(slI2C_NoNACKError);
#if showDebugDataI2C == 1
  showErrors("slI2C_ReadByte_NACK");
  slUART_WriteString("ReadByte_NACK:  ");
  slUART_LogBinary(TWDR);
#endif
  slI2C_readByteValue = TWDR;
  return I2C_Error;
}

uint8_t slI2C_ReadByte_ACK() {
  I2C_Error = 0;
  TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
  slI2C_WaitForComplete();
  if (TW_STATUS != TW_MR_DATA_ACK) slI2C_SetError(slI2C_NoACKError);
#if showDebugDataI2C == 1
  showErrors("slI2C_ReadByte_ACK");
  slUART_WriteString("ReadByte_ACK:  ");
  slUART_LogBinary(TWDR);
#endif
  slI2C_readByteValue = TWDR;
  return I2C_Error;
}

#endif
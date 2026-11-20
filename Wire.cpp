/*
  Wire.cpp - минимальная I2C библиотека для ADS1115 и LCD 1602 (PCF8574)
  Copyright (c) 2006 Nicholas Zambetti. All right reserved.
  База: MiniCore (Arduino 1.8.x), упрощено до master-only.

  Версия 6.5. См. полный комментарий в Wire.h.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

extern "C" {
  #include <inttypes.h>
  #include "utility/twi.h"
}

#include "Wire.h"

uint8_t TwoWire::rxBuffer[TWI_BUFFER_SIZE];
uint8_t TwoWire::rxBufferIndex  = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress      = 0;
uint8_t TwoWire::txBuffer[TWI_BUFFER_SIZE];
uint8_t TwoWire::txBufferLength = 0;

uint8_t TwoWire::transmitting = 0;

TwoWire::TwoWire() {}

void TwoWire::begin(void)
{
  rxBufferIndex = rxBufferLength = 0;
  txBufferLength = 0;
  twi_init();
}

void TwoWire::end(void) { twi_disable(); }

/* --- requestFrom ------------------------------------------------------ */

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  if (quantity > TWI_BUFFER_SIZE) quantity = TWI_BUFFER_SIZE;
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

/* --- write ------------------------------------------------------------ */

void TwoWire::beginTransmission(uint8_t address)
{
  transmitting = 1;
  txAddress = address;
  txBufferLength = 0;
}

uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, sendStop);
  txBufferLength = 0;
  transmitting = 0;
  return ret;
}

/* --- скорость ---------------------------------------------------------- */

void TwoWire::setClock(uint32_t frequency) { twi_setClock(frequency); }

/* --- Stream ----------------------------------------------------------- */

size_t TwoWire::write(uint8_t data)
{
  if (!transmitting) return 0;
  if (txBufferLength >= TWI_BUFFER_SIZE) { setWriteError(); return 0; }
  txBuffer[txBufferLength++] = data;
  return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  if (!transmitting) return 0;
  /* Прямой доступ к txBuffer — без вызова write(uint8_t) на каждый байт.
     Экономит несколько байт flash и пару десятков тактов на байт. */
  size_t i = 0;
  while (i < quantity && txBufferLength < TWI_BUFFER_SIZE)
    txBuffer[txBufferLength++] = data[i++];
  if (i < quantity) setWriteError();
  return i;
}

int TwoWire::available(void) { return rxBufferLength - rxBufferIndex; }

int TwoWire::read(void)
{
  if (rxBufferIndex < rxBufferLength) return rxBuffer[rxBufferIndex++];
  return -1;
}

int TwoWire::peek(void)
{
  if (rxBufferIndex < rxBufferLength) return rxBuffer[rxBufferIndex];
  return -1;
}

/* --- Хелперы ---------------------------------------------------------- */

uint8_t TwoWire::writeByte(uint8_t address, uint8_t data)
{
  return twi_writeTo(address, &data, 1, 1);
}

uint8_t TwoWire::writeReg16(uint8_t address, uint8_t reg, int16_t data)
{
  uint8_t buf[3] = { reg, (uint8_t)(data >> 8), (uint8_t)data };
  return twi_writeTo(address, buf, 3, 1);
}

TwoWire Wire = TwoWire();

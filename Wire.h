/*
  TwoWire.h - минимальная I2C библиотека для ADS1115 и LCD 1602 (PCF8574)
  Copyright (c) 2006 Nicholas Zambetti. All right reserved.
  База: MiniCore (Arduino 1.8.x), упрощено до master-only.

  Версия 6.4 — правки по code review:
  - BUFFER_LENGTH и I2C_BUFFER_LENGTH макросы для сторонних библиотек,
    ориентированных на оригинальный Arduino Wire.
  - setClock() документирован: при TWI_AUTO_SPEED=1 выключает авто-режим,
    что может поставить PCF8574 на 400 кГц (выше datasheet NXP).
  - twi_inRepStart проверка в setClock через twi.c.

  Версия 6.5 — принято из пользовательской переработки (база v6.1):
  - recover(): публичное освобождение залипшей шины (twi_recover());
    автоматически вызывается при таймауте транзакции (TWI_AUTO_RECOVER);
  - setClock(0) возвращает авто-режим скорости.

  Версия 6.4.1 — правки ревью v6.4:
  - BUFFER_LENGTH/I2C_BUFFER_LENGTH/WIRE_BUFFER_LENGTH под #ifndef guard
    (не конфликтовать с ядром, если оно тоже их определяет).

  Версия 6.1 — авто-скорость по адресу цели (ADS1115 400 кГц / PCF8574
  LCD 100 кГц), рабочая setClock(); одна сигнатура на метод, один индекс
  tx-буфера, перегрузки write() для int; int-перегрузки
  beginTransmission/requestFrom удалены — давали неоднозначность вызова.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <Arduino.h>
#include <inttypes.h>
#include "Stream.h"

#define WIRE_HAS_END 1

extern "C" {
  #include "utility/twi.h"
}
/* Размер буфера I2C — чтобы сторонние библиотеки могли проверить его
   на этапе компиляции (#if WIRE_BUFFER_LENGTH < N → #error). */
#ifndef WIRE_BUFFER_LENGTH
#define WIRE_BUFFER_LENGTH TWI_BUFFER_SIZE
#endif
/* Есть метод writeBuf(): одна транзакция из внешнего буфера. */
#define WIRE_HAS_WRITE_BUF 1
/* Есть метод recover(): освобождение залипшей шины (twi_recover).
   Автоматически вызывается при таймауте транзакции (TWI_AUTO_RECOVER). */
#define WIRE_HAS_RECOVER 1

/* Совместимость со сторонними библиотеками, ориентированными на
   оригинальный Arduino Wire (где буфер = 32 байт). Если библиотека
   пишет в Wire больше TWI_BUFFER_SIZE байт — лишнее молча отбрасывается,
   остаётся setWriteError(). Для надёжности добавляем макросы BUFFER_LENGTH
   и I2C_BUFFER_LENGTH, чтобы #if BUFFER_LENGTH < N мог сработать. */
#ifndef BUFFER_LENGTH
#define BUFFER_LENGTH        TWI_BUFFER_SIZE
#endif
#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH    TWI_BUFFER_SIZE
#endif

/* Совместимость с библиотеками, использующими API таймаута Arduino Wire 1.8+
   (setWireTimeout / getWireTimeoutFlag / clearWireTimeoutFlag).
   В этой сборке таймаут захардкожен через TWI_TIMEOUT_US в twi.h —
   setWireTimeout() — no-op, getWireTimeoutFlag() всегда false. */
#define WIRE_HAS_TIMEOUT 1

class TwoWire : public Stream
{
  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferLength;   /* он же индекс записи */

    static uint8_t transmitting;
  public:
    TwoWire();

    void begin();
    void end();

    /* Ручная смена скорости: 100000 -> 100 кГц, 400000 -> 400 кГц (от F_CPU).
       Округляется к ближайшей достижимой. При TWI_AUTO_SPEED=1 ВЫКЛЮЧАЕТ
       авто-режим — последующие транзакции к LCD (PCF8574) пойдут на заданной
       скорости, что может быть выше 100 кГц и не гарантировано datasheet NXP.
       Для надёжной работы с LCD оставьте авто-режим (не вызывайте setClock)
       или вызывайте setClock(100000).
       Идемпотентность: begin() НЕ сбрасывает ручную скорость —
       Wire.setClock(...); lcd.begin(); сохранит настройку.
       TWBR применяется перед очередным START — никогда не пишется
       посреди удержанной шины. Вернуть авто-режим: setClock(0). */
    void setClock(uint32_t frequency);

    /* Освободить шину, залипшую слэйвом (SDA удерживается низко):
       до 9 тактов SCL + STOP. Возвращает 1, если обе линии свободны.
       Сбрасывает tx-состояние. При таймауте транзакции вызывается
       автоматически (TWI_AUTO_RECOVER=1). */
    inline uint8_t recover(void)
    {
      transmitting = 0;
      txBufferLength = 0;
      return twi_recover();
    }

    /* === API таймаута (заглушки для совместимости) ==========================
       WIRE_HAS_TIMEOUT определён выше — сторонние библиотеки, проверяющие
       этот макрос, будут вызывать эти методы. В этой сборке таймаут
       захардкожен через TWI_TIMEOUT_US в twi.h — все три метода no-op. */
    inline void setWireTimeout(uint32_t timeout = 25000, bool reset_with_timeout = false)
    { (void)timeout; (void)reset_with_timeout; }
    inline bool getWireTimeoutFlag(void) { return false; }
    inline void clearWireTimeoutFlag(void) {}

    /* Одна сигнатура на метод: литералы (0x48, 2, true) приводятся к uint8_t сами.
       Парные перегрузки (int,int) давали неоднозначность на смешанных типах,
       например requestFrom(const uint8_t ADDR, 2). */
    void beginTransmission(uint8_t address);
    uint8_t endTransmission(uint8_t sendStop = 1);

    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop = 1);

    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *data, size_t quantity);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    using Print::write;

    /* Перегрузки для int-аргументов и литералов: без них Wire.write(0)
       неоднозначно с Print::write(const char*). */
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n)          { return write((uint8_t)n); }
    inline size_t write(unsigned int n)  { return write((uint8_t)n); }
    inline size_t write(int n)           { return write((uint8_t)n); }

    /* ADS1115: 16-бит big-endian из/в буфер. */
    inline int16_t read16(void)
    {
      if ((uint8_t)(rxBufferLength - rxBufferIndex) >= 2) {
        uint16_t v = ((uint16_t)rxBuffer[rxBufferIndex] << 8)
                   |  (uint16_t)rxBuffer[rxBufferIndex + 1];
        rxBufferIndex += 2;
        return (int16_t)v;
      }
      return 0;
    }
    /* Одна проверка transmitting и места в буфере вместо двух вызовов write(). */
    inline void write16(int16_t data)
    {
      if (!transmitting) return;
      if (txBufferLength + 2 > TWI_BUFFER_SIZE) { setWriteError(); return; }
      txBuffer[txBufferLength++] = (uint8_t)(data >> 8);
      txBuffer[txBufferLength++] = (uint8_t)(data & 0xFF);
    }

    /* Одна транзакция (START, адрес, данные, STOP) прямо из буфера вызывающего:
       без копирования в tx-буфер. length <= TWI_BUFFER_SIZE.
       sendStop=0 — оставить шину для repeated start (см. ADS1115).
       Возвращает код как endTransmission(): 0 ок, 1..6 ошибка (6 = STOP timeout).
       Проверяет address (<= 0x7F) и data (!= NULL при length > 0). */
    inline uint8_t writeBuf(uint8_t address, uint8_t *data, uint8_t length,
                            uint8_t sendStop = 1)
    {
      if (length > TWI_BUFFER_SIZE) return 1;
      if (address > 0x7F) return 4;
      if (length > 0 && data == 0) return 4;
      return twi_writeTo(address, data, length, sendStop);
    }

    /* PCF8574: один байт в один шаг. */
    uint8_t writeByte(uint8_t address, uint8_t data);

    /* ADS1115: [reg][hi][lo] в один шаг (запись конфига). */
    uint8_t writeReg16(uint8_t address, uint8_t reg, int16_t data);
};

extern TwoWire Wire;

#endif

/*
  twi.c - минимальный master-only TWI для ADS1115 + PCF8574 (LCD 1602)
  Copyright (c) 2006 Nicholas Zambetti. All right reserved.
  База: MiniCore (Arduino 1.8.x), упрощено до master-only.

  Версия 6.5 — принято из пользовательской переработки (см. twi.h):
  - twi_recover(): публичная функция восстановления шины; TWCR=0 до
    бит-бэнга и чтение TWI_PIN (фиксы v6.4.1 сохранены); ожидание
    отпускания SCL при clock stretching (до ~50 мкс); STOP только при
    свободном SCL; возвращает 1, если обе линии свободны.
  - twi_setClock(0) -> авто-режим; ручная скорость хранится в
    twi_manual_twbr и применяется в twi_pick_speed() перед START.
  - twi_init: всегда настраивает пины/TWSR, TWBR = manual ? manual_twbr
    : TWI_TWBR (прежний пропуск блока при manual-латче был хрупче).
  - twi_stop: поллинг TWSTO с проверкой таймера раз в 256 итераций,
    без _delay_us; util/delay.h не нужен.

  Версия 6.4.1 — правки ревью v6.4:
  - twi_reset: recover_bus выполняется при TWEN=0. При TWEN=1 модуль TWI
    сам управляет пинами SDA/SCL (override порта) — GPIO-перёдёргивание
    до шины не доходило, восстановление было нерабочим.
  - twi_recover_bus: уровень SDA читается из PIN, а не из PORT-латча
    (в латче после twi_init всегда 1 от подтяжки — цикл всегда выходил
    после первой итерации, 9 тактов SCL не генерировались).
  - twi_disable: сбрасывает twi_manual_speed — теперь заявленный в
    комментарии «полный сброс: twi_disable() + twi_init()» реально работает.

  Версия 6.4 — правки по code review:
  - twi_setClock: знаковая арифметика против переполнения при малом F_CPU.
  - twi_setClock: не пишет TWBR при удержанной шине (twi_inRepStart).
  - twi_init: идемпотентный — НЕ сбрасывает twi_manual_speed и TWBR, если
    они были заданы через twi_setClock ранее. Это позволяет скетчу
    Wire.setClock(...); lcd.begin(); не терять ручную скорость.
    Для полного сброса вызывайте twi_disable() + twi_init().
  - twi_reset: восстановление шины через toggle SCL до 9 раз, если slave
    завис с SDA=0 (например, после просадки питания LCD-модуля).
  - twi_stop возвращает код ошибки (6 = STOP timeout), пробрасывается в
    twi_writeTo/twi_readFrom.
  - Проверки аргументов: data==NULL при length>0, address <= 0x7F.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#include <inttypes.h>
#include <avr/io.h>
#include <compat/twi.h>
#include "Arduino.h"   /* для micros(), delayMicroseconds() */
#include "twi.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* Таймаут сравнивается в 16 битах (дешевле 32-битной арифметики). */
_Static_assert(TWI_TIMEOUT_US <= 0xFFFFul, "TWI_TIMEOUT_US must fit in 16 bits");

/* Внутренний "статус" при таймауте: реальный TW_STATUS кратен 8, 0xFF не бывает. */
#define TWI_ST_TIMEOUT 0xFF

/* Код ошибки twi_writeTo/twi_readFrom: таймаут STOP. Дополняет 0..5. */
#define TWI_ERR_STOP_TIMEOUT 6

/* Шина удержана после sendStop=0: TWBR трогать нельзя, twi_start сделает
   повторный START, twi_stop/twi_recover сбросят флаг. */
static volatile uint8_t twi_inRepStart;
/* Пользователь вызвал twi_setClock(f != 0) — авто-режим выключен до
   twi_setClock(0) или twi_disable(). */
static volatile uint8_t twi_manual_speed;
/* Значение TWBR ручного режима; применяется в twi_pick_speed() перед
   очередным START (запись TWBR посреди удержанной шины недопустима). */
static volatile uint8_t twi_manual_twbr;

/* --- восстановление шины ------------------------------------------------
   Слэйв (PCF8574, ADS1115) мог застрять посреди байта и держит SDA низко
   после сбоя питания или обрыва транзакции. Лечение: такты SCL — слэйв
   дошифровывает байт и отпускает SDA (9 тактов покрывают максимальный
   10-битный сдвиг), затем STOP, чтобы слэйвы вернулись в ожидание START.
   Публичная функция: вызывается из twi_reset() при таймауте
   (TWI_AUTO_RECOVER=1) или вручную (Wire.recover()).
   Если SCL удерживается низко слэйвом (clock stretching сверх нормы) —
   вернётся 0: программно не лечится, нужен сброс питания слэйва.
   TWBR не меняется (скорость сохраняется). Уровни читаются из TWI_PIN
   (физические линии), не из PORT-латча; TWCR=0 ОБЯЗАТЕЛЬНО до бит-бэнга:
   при TWEN=1 модуль сам управляет пинами и записи PORT/DDR не доходят. */
uint8_t twi_recover(void)
{
  TWCR = 0;                       /* модуль отключён — пинами управляет порт */
  twi_inRepStart = 0;

  /* Обе линии — входы с подтяжкой. */
  cbi(TWI_DDR, TWI_SDA_BIT);
  cbi(TWI_DDR, TWI_SCL_BIT);
  sbi(TWI_PORT, TWI_SDA_BIT);
  sbi(TWI_PORT, TWI_SCL_BIT);
  delayMicroseconds(TWI_RECOVER_HALF_US);

  /* До 9 тактов SCL, пока SDA не поднимется. */
  for (uint8_t i = 0; i < 9 && !(TWI_PIN & _BV(TWI_SDA_BIT)); i++) {
    /* SCL вниз (PORT гасим ДО перевода на выход — без глитча) */
    cbi(TWI_PORT, TWI_SCL_BIT);
    sbi(TWI_DDR,  TWI_SCL_BIT);
    delayMicroseconds(TWI_RECOVER_HALF_US);
    /* SCL отпустить и дождаться высокого уровня (растяжка до ~50 мкс) */
    cbi(TWI_DDR,  TWI_SCL_BIT);
    sbi(TWI_PORT, TWI_SCL_BIT);
    uint8_t w = 50;
    while (!(TWI_PIN & _BV(TWI_SCL_BIT)) && w--) delayMicroseconds(1);
    delayMicroseconds(TWI_RECOVER_HALF_US);
  }

  /* STOP: SDA низко при высоком SCL, затем SDA вверх. Только при свободном
   * SCL — иначе STOP не имеет смысла. */
  if (TWI_PIN & _BV(TWI_SCL_BIT)) {
    cbi(TWI_PORT, TWI_SDA_BIT);
    sbi(TWI_DDR,  TWI_SDA_BIT);
    delayMicroseconds(TWI_RECOVER_HALF_US);
    cbi(TWI_DDR,  TWI_SDA_BIT);
    sbi(TWI_PORT, TWI_SDA_BIT);
    delayMicroseconds(TWI_RECOVER_HALF_US);
  }

  uint8_t ok = ((TWI_PIN & _BV(TWI_SDA_BIT)) &&
                (TWI_PIN & _BV(TWI_SCL_BIT))) ? 1 : 0;

  TWSR = 0;                       /* на случай, если prescaler был тронут */
  TWCR = _BV(TWEN);               /* модуль включён обратно */
  return ok;
}

/* --- init / disable --------------------------------------------------- */

void twi_init(void)
{
  twi_inRepStart = 0;
  /* Латч manual НЕ сбрасываем: повторный begin() (например, lcd.begin()
     после Wire.setClock()) не должен терять выбранную скорость. Полный
     сброс — twi_disable() + twi_init(). */
  /* SDA/SCL как входы с подтяжкой. */
  cbi(TWI_DDR, TWI_SDA_BIT);
  cbi(TWI_DDR, TWI_SCL_BIT);
  sbi(TWI_PORT, TWI_SDA_BIT);
  sbi(TWI_PORT, TWI_SCL_BIT);

  TWSR = 0;              /* prescaler = 1 */
  TWBR = twi_manual_speed ? twi_manual_twbr : TWI_TWBR;
  TWCR = _BV(TWEN);      /* без TWEA/TWIE: модуль не отвечает как slave */
}

void twi_disable(void)
{
  TWCR = 0;
  twi_inRepStart = 0;
  twi_manual_speed = 0;  /* полный сброс: disable + init вернут авто-режим */
  cbi(TWI_PORT, TWI_SDA_BIT);
  cbi(TWI_PORT, TWI_SCL_BIT);
}

/* --- скорость --------------------------------------------------------- */

/* Ручная установка скорости: TWBR = (F_CPU/f - 16)/2, prescaler = 1.
   Округляется к ближайшей достижимой: 400 кГц -> 12, 100 кГц -> 72,
   200 кГц -> 32 @ 16 МГц. frequency == 0 возвращает авто-режим.
   Значение хранится в twi_manual_twbr и применяется в twi_pick_speed()
   перед очередным START, когда модуль гарантированно idle: запись TWBR
   посреди удержанной шины (rep-start) недопустима, поэтому прямой записи
   TWBR здесь нет и проверка twi_inRepStart не нужна.
   Знаковая арифметика: при F_CPU/f < 16 беззнаковая разность ушла бы в
   переполнение и дала бы самую МЕДЛЕННУЮ скорость вместо самой быстрой. */
void twi_setClock(uint32_t frequency)
{
  if (frequency == 0) { twi_manual_speed = 0; return; }

  if (frequency > 400000UL) frequency = 400000UL;
  if (frequency <  31000UL) frequency =  31000UL;
  long twbr = ((long)(F_CPU / frequency) - 16L) / 2L;
  if (twbr < 10)  twbr = 10;    /* минимум по даташиту */
  if (twbr > 255) twbr = 255;   /* максимум регистра */
  twi_manual_twbr = (uint8_t)twbr;
  twi_manual_speed = 1;
}

/* Выбор скорости перед START. Вызывается, когда модуль idle — запись
   TWBR безопасна. При удержанной шине (twi_inRepStart) не трогаем.
   Ручная скорость действует в обеих сборках (TWI_AUTO_SPEED 0 и 1). */
static void twi_pick_speed(uint8_t address)
{
  if (twi_inRepStart) { (void)address; return; }
  if (twi_manual_speed) { TWBR = twi_manual_twbr; return; }
#if TWI_AUTO_SPEED
  TWBR = TWI_ADDR_IS_SLOW(address) ? TWI_SLOW_TWBR : TWI_FAST_TWBR;
#else
  (void)address;   /* фиксированная TWI_TWBR (выставлена в twi_init) */
#endif
}

/* --- внутренние помощники ---------------------------------------------- */

/* Сброс модуля TWI. При TWI_AUTO_RECOVER=1 заодно пытаемся освободить
   шину: twi_recover() передёргивает SCL до 9 раз, если slave держит
   SDA=0, и сам включает модуль обратно. При 0 — рестарт модуля (v6.1). */
static void twi_reset(void)
{
#if TWI_AUTO_RECOVER
  twi_recover();
#else
  TWCR = 0;              /* TWEN=0 до любых манипуляций пинами */
  TWCR = _BV(TWEN);
#endif
  twi_inRepStart = 0;
}

/* Ждём TWINT. micros() проверяем раз в 256 итераций — чтобы не тормозить поллинг. */
static uint8_t twi_wait(uint16_t t0)
{
  uint8_t n = 0;
  while (!(TWCR & _BV(TWINT))) {
    if (++n == 0 &&
        (uint16_t)((uint16_t)micros() - t0) > (uint16_t)TWI_TIMEOUT_US)
      return 0;
  }
  return 1;
}

/* STOP; TWINT после него не ставится, ждём сброса TWSTO.
   t0 — время начала транзакции (НЕ STOP): таймаут общий на всю
   транзакцию. Поллинг без пауз, проверка таймера раз в 256 итераций
   (как в twi_wait) — без _delay_us на каждый круг.
   Возвращает 1 при успехе, 0 при таймауте — вызывающий мапит в свой
   код ошибки (writeTo -> 6 = STOP timeout, readFrom -> 0: данные
   не доверяются). */
static uint8_t twi_stop(uint16_t t0)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
  uint8_t n = 0;
  while (TWCR & _BV(TWSTO)) {
    if (++n == 0 &&
        (uint16_t)((uint16_t)micros() - t0) > (uint16_t)TWI_TIMEOUT_US) {
      twi_reset();    /* twi_reset/recover сбросят twi_inRepStart */
      return 0;       /* таймаут — шина не освободилась */
    }
  }
  twi_inRepStart = 0;
  return 1;           /* STOP завершён */
}

/* START (или повторный START, если шина удержана) + SLA+R/W. */
static uint8_t twi_start(uint8_t slarw, uint16_t t0)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
  if (!twi_wait(t0)) return TWI_ST_TIMEOUT;

  uint8_t s = TW_STATUS;
  if (s != TW_START && s != TW_REP_START) return s;

  TWDR = slarw;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twi_wait(t0)) return TWI_ST_TIMEOUT;
  return TW_STATUS;
}

/* --- readFrom --------------------------------------------------------- */

uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  if (length == 0) return 0;
  if (length > TWI_BUFFER_SIZE) return 0;
  if (address > 0x7F) return 0;
  if (data == 0) return 0;

  twi_pick_speed(address);
  uint16_t t0 = (uint16_t)micros();
  uint8_t s = twi_start((uint8_t)(address << 1) | TW_READ, t0);
  uint8_t n = 0;

  if (s == TWI_ST_TIMEOUT) { twi_reset(); return 0; }

  if (s == TW_MR_SLA_ACK) {
    do {
      TWCR = _BV(TWINT) | _BV(TWEN) | ((n + 1 < length) ? _BV(TWEA) : 0);
      if (!twi_wait(t0)) { twi_reset(); return 0; }
      s = TW_STATUS;
      if (s != TW_MR_DATA_ACK && s != TW_MR_DATA_NACK) break;
      data[n++] = TWDR;
    } while (n < length);
  }

  if (sendStop || n < length) {
    if (!twi_stop(t0)) return 0;   /* STOP-таймаут: данные не доверяются */
  } else {
    twi_inRepStart = 1;
  }
  return n;
}

/* --- writeTo ---------------------------------------------------------- */

uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  if (length > TWI_BUFFER_SIZE) return 1;
  if (address > 0x7F) return 4;
  if (length > 0 && data == 0) return 4;

  twi_pick_speed(address);
  uint16_t t0 = (uint16_t)micros();
  uint8_t s = twi_start((uint8_t)(address << 1) | TW_WRITE, t0);
  uint8_t ret;

  if (s == TWI_ST_TIMEOUT) { twi_reset(); return 5; }

  if (s != TW_MT_SLA_ACK) {
    ret = (s == TW_MT_SLA_NACK) ? 2 : 4;
  } else {
    ret = 0;
    while (length--) {
      TWDR = *data++;
      TWCR = _BV(TWINT) | _BV(TWEN);
      if (!twi_wait(t0)) { twi_reset(); return 5; }
      s = TW_STATUS;
      if (s != TW_MT_DATA_ACK) { ret = (s == TW_MT_DATA_NACK) ? 3 : 4; break; }
    }
  }

  if (ret || sendStop) {
    /* Ошибка STOP не скрывается: шина не освободилась, следующая
       транзакция зависла бы. Код 6 = STOP timeout (см. twi.h/Wire.h). */
    if (!twi_stop(t0)) return TWI_ERR_STOP_TIMEOUT;
  } else {
    twi_inRepStart = 1;
  }
  return ret;
}

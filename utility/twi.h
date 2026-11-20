/*
  twi.h - минимальный master-only TWI для ADS1115 + PCF8574
  Copyright (c) 2006 Nicholas Zambetti. All right reserved.
  База: MiniCore (Arduino 1.8.x), упрощено до master-only.

  Версия 6.5 — принято из пользовательской переработки (база v6.1):
  - twi_recover() — публичное восстановление залипшей шины (до 9 тактов
    SCL + STOP, ожидание отпускания SCL при clock stretching, STOP только
    при свободном SCL); вызывается автоматически из twi_reset() при
    любом таймауте (TWI_AUTO_RECOVER=1) или вручную через Wire.recover().
  - twi_setClock(0) возвращает авто-режим скорости.
  - ручная скорость хранится в twi_manual_twbr и применяется в
    twi_pick_speed() перед очередным START — запись TWBR посреди
    удержанной шины невозможна by design (проверка inRepStart в
    setClock больше не нужна).
  - TWI_TWBR_FOR: добавлен ВЕРХНИЙ кламп 255 (при очень малых f раньше
    было тихое переполнение uint8_t); TWI_TWBR по умолчанию — от F_CPU.
  - twi_recover/twi_stop не используют util/delay.h — только
    delayMicroseconds() из Arduino.h.

  Версия 6.4.1 — правки ревью v6.4:
  - добавлен TWI_PIN: состояние SDA в recover_bus читается из PIN-регистра,
    а не из PORT-латча (BUG: цикл восстановления всегда выходил сразу);
  - исправлена таблица частот в комментарии (цифры, код не менялся);
  - уточнён комментарий TWI_TWBR: в авто-режиме он не используется.

  Версия 6.4 — правки по code review:
  - TWBR вычисляется через F_CPU (TWI_TWBR_FOR макрос), не привязан к 16 МГц.
    Корректно работает на 8/16/20 МГц MiniCore.
  - TWI_ADDR_IS_SLOW сужен до 0x27/0x3F (типовые LCD-модули PCF8574/PCF8574A).
    Раньше захватывал 0x20..0x27 (MCP23017), 0x38 (AHT10/20), 0x3C/0x3D (SSD1306) —
    все они молча ехали на 100 кГц.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#ifndef twi_h
#define twi_h

  #include <inttypes.h>
  #include <stdbool.h>

  /* === Вычисление TWBR от F_CPU =========================================
     SCL = F_CPU / (16 + 2*TWBR), prescaler = 1.
     Даташит требует TWBR >= 10 в режиме master.

     16 МГц: TWBR=12 -> 400 кГц;  TWBR=72 -> 100 кГц.
     20 МГц: TWBR=12 -> 500 кГц (выше Fast Mode!), TWBR=42 -> 200 кГц, TWBR=92 -> 100 кГц.
      8 МГц: TWBR=10 -> 222 кГц (максимум по даташиту), TWBR=32 -> 100 кГц, TWBR=72 -> 50 кГц.

     Чтобы задать частоту f, TWI_TWBR_FOR(f) = (F_CPU/f - 16)/2,
     clamped к [10..255] (нижний — минимум даташита, верхний — регистр;
     без верхнего клампа малые f давали тихое переполнение uint8_t).
     Для 20 МГц при f=400 кГц: (50-16)/2 = 17 -> ровно 400 кГц (20е6/(16+2*17)).
     Для  8 МГц при f=400 кГц: (20-16)/2 = 2 -> clamped до 10 -> 222 кГц. */
  #define TWI_TWBR_RAW(f)   (((long)((F_CPU) / (f)) - 16L) / 2L)
  #define TWI_TWBR_CLAMP(v) ((v) < 10L ? 10L : ((v) > 255L ? 255L : (v)))
  #define TWI_TWBR_FOR(f)   ((uint8_t)TWI_TWBR_CLAMP(TWI_TWBR_RAW(f)))

  /* Базовый TWBR: начальное значение в twi_init() для ручного режима
     (TWI_AUTO_SPEED=0) до первого twi_setClock(). В авто-режиме НЕ
     используется — там свои TWI_FAST_TWBR/TWI_SLOW_TWBR. Чтобы изменить
     скорости авто-режима: -DTWI_FAST_TWBR=32 -DTWI_SLOW_TWBR=72. */
  #ifndef TWI_TWBR
  #define TWI_TWBR TWI_TWBR_FOR(400000UL)
  #endif

  /* --- Две скорости на одной шине: ADS1115 400 кГц, LCD 100 кГц --------
     PCF8574 (модуль LCD 1602) по datasheet NXP гарантирован только до 100 кГц.
     ADS1115 тянет 400 кГц (Fast Mode).

     TWI_AUTO_SPEED=1 (по умолчанию): перед КАЖДОЙ транзакцией скорость
       выбирается по адресу цели: TWI_ADDR_IS_SLOW(a) ? TWI_SLOW_TWBR :
       TWI_FAST_TWBR.
     TWI_AUTO_SPEED=0: фиксированная TWI_TWBR; ручная смена через
       twi_setClock() / Wire.setClock() между транзакциями.
     ВНИМАНИЕ: Wire.setClock(f), f != 0, выключает авто-режим — ВСЕ
       устройства, включая PCF8574 (гарантирован только до 100 кГц),
       поедут на этой частоте. Вернуть авто-режим: Wire.setClock(0). */
  #ifndef TWI_AUTO_SPEED
  #define TWI_AUTO_SPEED 1
  #endif

  /* Быстрая скорость: ADS1115 и всё прочее. По умолчанию 400 кГц от F_CPU. */
  #ifndef TWI_FAST_TWBR
  #define TWI_FAST_TWBR TWI_TWBR_FOR(400000UL)
  #endif

  /* Медленная скорость: PCF8574 / LCD 1602. 100 кГц от F_CPU. */
  #ifndef TWI_SLOW_TWBR
  #define TWI_SLOW_TWBR TWI_TWBR_FOR(100000UL)
  #endif

  /* Адреса, которые едут на медленной скорости (7-битные, как в
     twi_writeTo/twi_readFrom).
     По умолчанию — ТОЛЬКО типовые адреса LCD-модулей PCF8574/PCF8574A:
       0x27 — типовой PCF8574 (модули с меткой "0x27")
       0x3F — типовой PCF8574A (китайские модули с меткой "0x3F")
     НЕ включает 0x20..0x26 (MCP23017 и др.), 0x38 (AHT10/20), 0x3C/0x3D (SSD1306).

     Если у вас другой адрес LCD или расширенная шина — переопределите:
       #define TWI_ADDR_IS_SLOW(a) ((a) == 0x27 || (a) == 0x3F)  — несколько адресов
       #define TWI_ADDR_IS_SLOW(a) ((a) >= 0x20 && (a) <= 0x27)  — весь диапазон PCF8574
       #define TWI_ADDR_IS_SLOW(a) ((uint8_t)((a) & 0xF8) == 0x20 \
                                || (uint8_t)((a) & 0xF8) == 0x38) — вся линейка PCF8574/A
       #define TWI_ADDR_IS_SLOW(a) (0)                            — всё быстро */
  #ifndef TWI_ADDR_IS_SLOW
  #define TWI_ADDR_IS_SLOW(a) ((a) == 0x27 || (a) == 0x3F)
  #endif

  /* 5 байт: ADS1115 запись [reg][hi][lo]=3, чтение 2; PCF8574 — 1;
     пакет LCD 1602 на символ: [hi|E][hi][lo|E][lo] = 4, а с байтом
     предустановки RS (LCD_RS_SETUP_BYTE=1) [hi][hi|E][hi][lo|E][lo] = 5. */
  #ifndef TWI_BUFFER_SIZE
  #define TWI_BUFFER_SIZE 5
  #endif

  /* Таймаут в мкс на всю транзакцию (SMBus max clock stretching = 25 мс).
     Должен быть <= 65535 (сравнение в 16 битах). */
  #ifndef TWI_TIMEOUT_US
  #define TWI_TIMEOUT_US 25000ul
  #endif


  /* Автоматическое восстановление шины при таймауте: twi_reset() вызывает
     twi_recover() (до 9 тактов SCL + STOP, ~100 мкс на сбой) вместо
     простого рестарта модуля. 0 — рестарт модуля без тактов (как в v6.1). */
  #ifndef TWI_AUTO_RECOVER
  #define TWI_AUTO_RECOVER 1
  #endif

  /* Полупериод тактов SCL при восстановлении, мкс (5 -> ~100 кГц). */
  #ifndef TWI_RECOVER_HALF_US
  #define TWI_RECOVER_HALF_US 5
  #endif

  /* Пины TWI на ATmega328P: SDA=PC4 (A4), SCL=PC5 (A5).
     Для ATmega324/644/1284 (40-pin) SDA=PC1, SCL=PC0 — переопределите макросы.
     Для ATmega2560 и др. — свой порт, тоже переопределите. */
  #ifndef TWI_PORT
  #define TWI_PORT PORTC
  #endif
  #ifndef TWI_DDR
  #define TWI_DDR  DDRC
  #endif
  /* PIN-регистр того же порта: физический уровень SDA в recover_bus
     (чтение PORT даёт выходной латч, а не линию). */
  #ifndef TWI_PIN
  #define TWI_PIN  PINC
  #endif
  #ifndef TWI_SDA_BIT
  #define TWI_SDA_BIT 4
  #endif
  #ifndef TWI_SCL_BIT
  #define TWI_SCL_BIT 5
  #endif

  void    twi_init(void);
  void    twi_disable(void);
  /* Ручная установка скорости: 100000 -> 100 кГц, 400000 -> 400 кГц (от F_CPU).
     Округляется к ближайшей достижимой. frequency != 0 выключает авто-режим
     до twi_setClock(0) или twi_disable(); frequency == 0 возвращает авто-режим.
     Значение применяется в twi_pick_speed() перед очередным START — TWBR
     никогда не пишется посреди удержанной шины (rep-start). */
  void    twi_setClock(uint32_t frequency);
  /* Освободить шину, залипшую слэйвом (SDA удерживается низко): до 9 тактов
     SCL, пока SDA не поднимется, затем STOP, затем модуль TWI включается
     обратно. Возвращает 1, если обе линии свободны, иначе 0. */
  uint8_t twi_recover(void);
  uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop);
  uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop);

#endif

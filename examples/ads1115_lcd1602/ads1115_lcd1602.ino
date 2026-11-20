/*
  Пример использования оптимизированной библиотеки Wire (на базе MiniCore)
  с ADS1115 (16-бит АЦП) и LCD 1602 через PCF8574 (I2C-модуль).

  Подключение (Arduino Uno/Nano):
    A4 (SDA) -> SDA ADS1115 + SDA PCF8574
    A5 (SCL) -> SCL ADS1115 + SCL PCF8574
    VCC      -> VCC обоих модулей (5 В)
    GND      -> GND обоих модулей

  Адреса I2C (по умолчанию):
    ADS1115     : 0x48 (ADDR -> GND)
    PCF8574 LCD : 0x27 (часто 0x3F на китайских модулях — проверить!)

  Что демонстрирует пример:
  1) repeated start при чтении регистра ADS1115 (правильный паттерн datasheet):
       write(pointer, sendStop=false) -> requestFrom(addr, 2, sendStop=true)
  2) хелпер Wire.writeReg16() для записи [reg][hi][lo] в один шаг (запись конфига);
  3) хелпер Wire.read16() для чтения 16-бит big-endian значения;
  4) хелпер Wire.writeByte() для быстрой записи одного байта в PCF8574
     (минует beginTransmission/endTransmission, экономит несколько вызовов).

  Частота I2C — авто-переключение по адресу цели (twi.h, TWI_AUTO_SPEED=1):
    ADS1115 (0x48)      -> 400 кГц (TWI_FAST_TWBR=12)
    LCD 1602 (PCF8574)  -> 100 кГц (TWI_SLOW_TWBR=72), в скетче адрес 0x3F
  Таймаут 25 мс — TWI_TIMEOUT_US. Оба — compile-time, см. setup().
*/

#include "Wire.h"

// === Адреса I2C ===
#define ADS1115_ADDRESS   0x48
#define LCD_I2C_ADDRESS  0x27  // поменять на 0x3F если экран не отвечает

// === Регистры ADS1115 ===
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

// === Конфигурация ADS1115 (16 бит) ===
// [OS=1][MUX=000 AIN0-GND][PGA=001 4.096V][MODE=1 single-shot][DR=100 128SPS][COMP_QUE=11]
#define ADS1115_CONFIG  0b1000001110000011  // 0x8383

// === LCD 1602 (HD44780) через PCF8574 ===
// Распиновка типового I2C-модуля:
//   P0 -> RS, P1 -> RW, P2 -> EN, P3 -> BL (подсветка)
//   P4..P7 -> D4..D7 (4-битный режим)
#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08

uint8_t lcd_backlight = LCD_BL;

// Строб EN: поднять EN, подождать, опустить EN
static void lcd_strobe(uint8_t data)
{
  Wire.writeByte(LCD_I2C_ADDRESS, data | LCD_EN | lcd_backlight);
  delayMicroseconds(1);
  Wire.writeByte(LCD_I2C_ADDRESS, (data & ~LCD_EN) | lcd_backlight);
  delayMicroseconds(50);
}

// Отправка 4-битного nibble (старшие полбайта в P4-P7)
static void lcd_send4(uint8_t nibble, uint8_t rs)
{
  uint8_t data = (nibble << 4) | rs;
  lcd_strobe(data);
}

// Отправка полного байта в 2 этапа (4-битный режим HD44780)
static void lcd_send(uint8_t byte, uint8_t rs)
{
  lcd_send4(byte >> 4,   rs);
  lcd_send4(byte & 0x0F, rs);
}

// Инициализация LCD 1602 в 4-битном режиме
static void lcd_init(void)
{
  delay(50); // ожидание после подачи питания
  lcd_send4(0x03, 0); delay(5);
  lcd_send4(0x03, 0); delay(5);
  lcd_send4(0x03, 0); delayMicroseconds(150);
  lcd_send4(0x02, 0); // переход в 4-битный режим
  lcd_send(0x28, 0);  // Function set: 4-bit, 2 строки, 5x8
  lcd_send(0x0C, 0);  // дисплей ON, курсор OFF
  lcd_send(0x06, 0);  // вход: инкремент, без сдвига
  lcd_send(0x01, 0);  // очистка
  delay(2);
}

// Установка позиции курсора (0,0 .. 1,15)
static void lcd_setCursor(uint8_t col, uint8_t row)
{
  uint8_t addr = (row == 0) ? 0x00 : 0x40;
  addr += col;
  lcd_send(0x80 | addr, 0);
}

// Печать строки (дополнение пробелами не делается — caller управляет)
static void lcd_print(const char *s)
{
  while (*s) {
    lcd_send((uint8_t)*s++, LCD_RS);
  }
}

/*
 * Чтение 16-битного регистра ADS1115 через REPEATED START.
 * Это "правильный" паттерн из datasheet ADS1115:
 *   1) beginTransmission; write(reg); endTransmission(false)  — без STOP
 *   2) requestFrom(addr, 2, sendStop=true)                    — REPEATED START + чтение + STOP
 * Repeated start гарантирует, что между записью указателя регистра и чтением
 * другой master не вклинится и не сменить указатель.
 */
static int16_t ads1115_readReg(uint8_t reg)
{
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);  // sendStop=false -> готовим repeated start
  // Явный cast — иначе неоднозначность перегрузок requestFrom
  Wire.requestFrom((uint8_t)ADS1115_ADDRESS, (uint8_t)2, (uint8_t)true);
  return Wire.read16();
}

/*
 * Запись конфигурации ADS1115.
 * Используем специализированный хелпер writeReg16 (3-байт транзакция:
 * [reg][hi][lo]) — без цикла копирования, без проверки length+1 > buffer.
 * Эквивалентно:
 *   Wire.beginTransmission(ADS1115_ADDRESS);
 *   Wire.write(ADS1115_REG_CONFIG);
 *   Wire.write16(cfg);
 *   Wire.endTransmission(true);
 */
static void ads1115_writeConfig(int16_t cfg)
{
  Wire.writeReg16(ADS1115_ADDRESS, ADS1115_REG_CONFIG, cfg);
}

void setup(void)
{
  Wire.begin();
  /* Скорость выбирается автоматически по адресу цели (twi.h, по умолчанию
     TWI_AUTO_SPEED=1): типовые адреса LCD-модулей PCF8574/PCF8574A (0x27/0x3F)
     — 100 кГц (TWI_SLOW_TWBR=72), всё остальное, в т.ч. ADS1115 (0x48) —
     400 кГц (TWI_FAST_TWBR=12). Ни скетч, ни драйвер LCD переключать не
     должны. Если LCD на другом адресе — переопределите TWI_ADDR_IS_SLOW.

     Ручной режим: -DTWI_AUTO_SPEED=0 — тогда скорость фиксированная
     TWI_TWBR, менять можно на лету: Wire.setClock(400000) / (100000).
     ВНИМАНИЕ: setClock(f != 0) выключает авто-режим для ВСЕЙ шины
     (PCF8574 выше 100 кГц не гарантирован); вернуть авто-режим —
     Wire.setClock(0). Значение применяется перед очередным START.

     Восстановление залипшей шины: при таймауте вызывается автоматически
     (TWI_AUTO_RECOVER=1); вручную — Wire.recover() (до 9 тактов SCL + STOP,
     вернёт 1, если шина свободна).

     Таймаут 25 мс (SMBus) задан через TWI_TIMEOUT_US; переопределение:
       -DTWI_TIMEOUT_US=50000ul
     Для 8 МГц: 400 кГц недостижимы — все дефолтные скорости считаются от
     F_CPU: TWBR=10 даст ~222 кГц (максимум по даташиту), 100 кГц -> 32. */

  lcd_init();
  lcd_setCursor(0, 0);
  lcd_print("ADS1115 ready");
  delay(500);
  lcd_send(0x01, 0); // очистка
  delay(2);
}

void loop(void)
{
  // 1) Стартуем одиночное преобразование (OS=1 в старшем байте конфига)
  ads1115_writeConfig((int16_t)ADS1115_CONFIG);

  // 2) Ждём завершения (бит OS сбрасывается в 0 при готовности).
  //    При 128 SPS преобразование занимает ~7.8 мс. Опрашиваем каждые 2 мс.
  int16_t cfg;
  uint8_t guard = 0;
  do {
    delay(2);
    cfg = ads1115_readReg(ADS1115_REG_CONFIG);
    if (++guard > 100) break; // защита ~200 мс
  } while ((cfg & 0x8000) == 0);

  // 3) Читаем результат (через repeated start)
  int16_t raw = ads1115_readReg(ADS1115_REG_CONVERSION);

  // 4) Перевод в милливольты: GAIN = 4.096 В, full-scale = ±32767.
  //    4096 мВ / 32768 = 0.125 мВ/LSB. Используем int32, чтобы не переполниться.
  int32_t millivolts = ((int32_t)raw * 4096) / 32768;

  // 5) Вывод на LCD
  char line[17];
  snprintf(line, sizeof(line), "U = %5ld mV   ", (long)millivolts);
  lcd_setCursor(0, 0);
  lcd_print(line);
  snprintf(line, sizeof(line), "raw = %6d   ", (int)raw);
  lcd_setCursor(0, 1);
  lcd_print(line);

  delay(1000);
}

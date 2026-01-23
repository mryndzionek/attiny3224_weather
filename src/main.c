#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <u8g2.h>
#include <u8x8_tinyavr_2.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "config.h"

#define DISPLAY_I2C_ADDR (0x78)

#define RED_LED_PIN PIN2_bm  // PB2
#define BUTTON_PIN PIN6_bm   // PA6
#define OWIRE_PIN PIN7_bm    // PA7

#define EV_TEMP_TIMEOUT _BV(0)
#define EV_BUTTON_PRESS _BV(1)
#define EV_TICK _BV(2)

#define BATTERY_LEVEL_NUM (4)
#define NUM_BATTERIES (3)

#define g_events (GPIOR0)
#define events (GPIOR1)

#define BIG_LABEL_FONT (u8g2_font_profont15_tf)
#define MEDIUM_LABEL_FONT (u8g2_font_5x7_mr)
#define SMALL_LABEL_FONT (u8g2_font_5x7_mr)

#define DS18B20_CONVERT_T (0x44)
#define DS18B20_READ (0xBE)

#define ONEWIRE_PIN_INPUT() (PORTA.DIR &= ~OWIRE_PIN)
#define ONEWIRE_PIN_OUTPUT() (PORTA.DIR |= OWIRE_PIN)
#define ONEWIRE_PIN_LOW() (PORTA.OUTCLR = OWIRE_PIN)
#define ONEWIRE_PIN_HIGH() (PORTA.OUTSET = OWIRE_PIN)
#define ONEWIRE_PIN_READ() (PORTA.IN & OWIRE_PIN)
#define ONEWIRE_RESET_RETRIES_MAX (128)
#define ONEWIRE_SKIP_ROM (0xCC)

#define DATA_BUF_SIZE (60U * 24 / 10)

#define I2C_DIR_WRITE (0)
#define I2C_DIR_READ (1)
#define I2C_FAILED (255)
#define I2C_SUCCEEDED (0)

#define AHT20_I2C_ADDR (0x38)
#define BMP280_I2C_ADDR (0x77)

#define dig_T1 (bmp280_calibration_data[0])
#define dig_T2 (bmp280_calibration_data[1])
#define dig_T3 (bmp280_calibration_data[2])

#define dig_P1 (bmp280_calibration_data[3])
#define dig_P2 (bmp280_calibration_data[4])
#define dig_P3 (bmp280_calibration_data[5])
#define dig_P4 (bmp280_calibration_data[6])
#define dig_P5 (bmp280_calibration_data[7])
#define dig_P6 (bmp280_calibration_data[8])
#define dig_P7 (bmp280_calibration_data[9])
#define dig_P8 (bmp280_calibration_data[10])
#define dig_P9 (bmp280_calibration_data[11])

static int16_t bmp280_calibration_data[12];

// clang-format off
// bdfconv -m '48-57,45,37,67,46' -b1 -o main_font.c -f1 -n main_font
const uint8_t main_font[805] U8G2_FONT_SECTION("main_font") = 
  "\16\1\5\5\5\6\1\1\6\23%\2\0%\367%\0\0\0\0\0\3\10%P\270\34/\205)\316Q"
  "\12\203\24\342$\243\60\204 Fq\210@\210\2!\243H\307 V\61\212YLb\226d\61\213I\314"
  "\322$fI\26\263\230\304,Mb\226\10\241\212q\244B$\210\30\311)\306\350\24\203\10\304!Hb"
  "\212r\234\302\24\5\0-\17\270\34\377\340\377\257\71\312\355\203\177\16\0.\17\256\334\375\340\377\243\302\42"
  "\24I\15\6\0\60^\270\34O\6TX\263\234\4\35\303\24\247@\205)Pa\210T\30B\21\206P"
  "\204!\24a\10E\30B\21\206P\204!\24a\10E\30B\21\206P\204!\24a\10E\30B\21\206"
  "P\204!\24a\10E\30B\21\206P\204!\24a\210T\230\2\25\246@\205\61L\201\240\344,\246)"
  "\320H\0\61\31\270\34g\305,I#\36\20yio\20\304\13\10A\311\377\377\177\7\0\62\77\270\34"
  "O\6T\32\263\234\4\35\303\24\247@\205)Pa\10E\30B\21\206H\205)Pa\10d\234@\25"
  "\322\210\206,\244\21\15YHC\226\244!\13iDC\26\322\220%iH\311\310\33\0\63\61\270\34\37"
  "\222\221'\215h\310B\32\321\220\205\64\242!\13iH\4&O\211\310,h\61K\224\274YB\303\61"
  "\214Q\214\23\241G\65ly\206\2\0\64\60\270\34O\305,d\61K\223\230e\223\230e\223\230e\223"
  "\230\305 X\61\212U\214b\21\244P\5)TA\12U\220BIF\336+f\371\16\0\65\63\270\34"
  "\37\222\221g\10J~\302`\216z\22\204\242c\230\342\24\250\60\5*\14\241\10J~\206P\204)P"
  "a\12T\30\303\24\10J\316b\232\2\215\4\0\66M\270\34O\6TX\263\234\4\35\303\24\247@\205"
  ")Pa\210T\30\202\222\27\16\346\250'A(:\206)N\201\12S\240\302\20\251\60D*\14\221\12"
  "C\244\302\20\251\60D*\14\221\12S\240\302\24\250\60\206)\20\224\234\305\64\5\32\11\0\67\63\270\34"
  "\37\222\221g\210T\30\2\31\206@\305!P!M\262\230\205\64\311b\26\322\220\305,IC\26\263$"
  "\15Y\314B\232d\61\13i\222\305,\244\261\0\70V\270\34O\6T\32\263\234\4\35\303\24\247@\205"
  ")Pa\10E\30B\21\206P\204!\24a\210T\230\2\25\246\70&b (\71\213YN\202PQ"
  "\214S\240\302\24\250\60D*\14\241\10C(\302\20\212\60\204\42L\201\12S\240\342\24\246@Pr\26"
  "\323\24h$\0\71J\270\34O\6TX\263\234\4\35\303\24\247@\205)Pa\10E\30B\21\206P"
  "\204!\24a\10E\30B\21\206P\204)Pa\12T\30\323\211\20L\71\314\20\4%\317\20\212\60\5"
  "*L\201\12c\230\2A\311YLS\240\221\0C.\270\34O\6TX\263\234\4\35\303\24\247@\205"
  ")Pa\210T\30\202\222\377\377\245\302\24\250\60\5*\214a\12\4%g\61M\201F\2\0\0\0\0"
  "\4\377\377\0";
// clang-format on

static u8g2_t u8g2;

ISR(RTC_PIT_vect) {
  static uint16_t tmout;
  static bool btn_state;
  static uint8_t init = 10;

  RTC.PITINTFLAGS = RTC_PI_bm;
  tmout++;
  g_events |= EV_TICK;
  const uint16_t tv = init ? 1 : 60;
  if (tmout == (tv * 32)) {
    tmout = 0;
    g_events |= EV_TEMP_TIMEOUT;
    if (init > 0) {
      init--;
    }
  }

  if (!btn_state) {
    if (PORTA.IN & BUTTON_PIN) {
      g_events |= EV_BUTTON_PRESS;
      btn_state = true;
    }
  } else {
    if (!(PORTA.IN & BUTTON_PIN)) {
      btn_state = false;
    }
  }
}

static void configure_pins(void) {
  PORTA.DIR = 0;
  PORTA.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN2CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN5CTRL |= PORT_PULLUPEN_bm;
  // PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
  // PORTA.PIN7CTRL |= PORT_PULLUPEN_bm;

  PORTB.DIR = 0;
  PORTB.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN2CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN3CTRL |= PORT_PULLUPEN_bm;
}

static inline void configure_rtc(void) {
  while (RTC.STATUS > 0) {
    ; /* Wait for all register to be synchronized */
  }

  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
}

static inline void configure_led(void) {
  PORTB.DIRSET = RED_LED_PIN;
  PORTB.OUTSET = RED_LED_PIN;
}

static inline void configure_button(void) {
  PORTA.DIRCLR = BUTTON_PIN;           // input
  PORTA.PIN1CTRL |= PORT_PULLUPEN_bm;  // with pullup
}

static inline void configure_watchdog(void) {
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_1KCLK_gc);
  while (WDT.STATUS & WDT_SYNCBUSY_bm) {
    ;
  }
}

static uint8_t onewire_reset(void) {
  uint8_t retval, retries;

  ONEWIRE_PIN_LOW();
  ONEWIRE_PIN_INPUT();

  retries = ONEWIRE_RESET_RETRIES_MAX;
  while (!ONEWIRE_PIN_READ()) {
    if (--retries == 0) {
      return (2);
    }
    _delay_us(1);
  }

  ONEWIRE_PIN_OUTPUT();
  _delay_us(480);
  ONEWIRE_PIN_INPUT();
  _delay_us(66);
  retval = ONEWIRE_PIN_READ();
  _delay_us(414);

  return (retval);
}

static uint8_t onewire_bit(uint8_t value) {
  cli();
  ONEWIRE_PIN_OUTPUT();
  _delay_us(1);
  if (value) {
    ONEWIRE_PIN_INPUT();
  }
  _delay_us(14);
  value = !(ONEWIRE_PIN_READ() == 0);
  _delay_us(45);
  ONEWIRE_PIN_INPUT();
  sei();

  return value;
}

static uint8_t onewire_write(uint8_t value) {
  uint8_t i, r;

  for (i = 0; i < 8; ++i) {
    r = onewire_bit(value & 0x01);
    value >>= 1;
    if (r) {
      value |= 0x80;
    }
  }

  return value;
}

uint8_t onewire_read(void) { return onewire_write(0xff); }

static int16_t read_ext_temp(void) {
  uint8_t msb, lsb;
  int8_t sign = 1;
  uint16_t t;

  onewire_reset();
  onewire_write(ONEWIRE_SKIP_ROM);
  onewire_write(DS18B20_CONVERT_T);

  onewire_reset();
  onewire_write(ONEWIRE_SKIP_ROM);
  onewire_write(DS18B20_READ);

  lsb = onewire_read();
  msb = onewire_read();
  t = ((uint16_t)msb << 8) | lsb;

  if ((msb & 0xf8) == 0xf8) {
    t = (65536 - t);
    sign = -1;
  }

  return sign * (((uint16_t)t * 10U) / 16U);
}

static uint8_t i2c_start(uint8_t address) {
  TWI0.MADDR = address;
  while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));

  if (TWI0.MSTATUS & TWI_ARBLOST_bm) {
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc));
    return I2C_FAILED;
  } else if (TWI0.MSTATUS & TWI_RXACK_bm) {
    TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc));
    return I2C_FAILED;
  }
  return I2C_SUCCEEDED;
}

static inline void i2c_stop(void) {
  TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
  while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc));
}

static uint8_t i2c_write(uint8_t byte) {
  TWI0.MDATA = byte;
  TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
  while (!(TWI0.MSTATUS & TWI_WIF_bm))
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) return I2C_FAILED;
  return !(TWI0.MSTATUS & TWI_RXACK_bm);
}

static uint8_t i2c_read(uint8_t ack) {
  uint8_t byte;
  while (!(TWI0.MSTATUS & TWI_RIF_bm));
  byte = TWI0.MDATA;
  if (ack)
    TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc | TWI_ACKACT_ACK_gc;
  else
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc;
  return byte;
}

static uint8_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t* val) {
  uint8_t ret = i2c_start((addr << 1) | 0x00);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ret = i2c_write(reg);
  if (ret == 0) {
    return I2C_FAILED;
  }
  ret = i2c_start((addr << 1) | 0x01);
  if (ret != I2C_SUCCEEDED) {
    return I2C_FAILED;
  }
  *val = i2c_read(0);
  i2c_stop();
  return I2C_SUCCEEDED;
}

static uint8_t i2c_read_register16(uint8_t addr, uint8_t reg, uint16_t* val) {
  uint8_t ret = i2c_start((addr << 1) | 0x00);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ret = i2c_write(reg);
  if (ret == 0) {
    return I2C_FAILED;
  }
  ret = i2c_start((addr << 1) | 0x01);
  if (ret != I2C_SUCCEEDED) {
    return I2C_FAILED;
  }
  const uint8_t b1 = i2c_read(1);
  const uint8_t b2 = i2c_read(0);
  i2c_stop();
  *val = ((uint16_t)b2 << 8) | b1;
  return I2C_SUCCEEDED;
}

static uint8_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t ret = i2c_start((addr << 1) | 0x00);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ret = i2c_write(reg);
  if (ret == 0) {
    return I2C_FAILED;
  }
  ret = i2c_write(val);
  if (ret == 0) {
    return I2C_FAILED;
  }
  i2c_stop();
  return I2C_SUCCEEDED;
}

static uint8_t i2c_write_data(uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t ret = i2c_start((addr << 1) | 0x00);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  for (uint8_t i = 0; i < len; i++) {
    ret = i2c_write(data[i]);
    if (ret == 0) {
      return I2C_FAILED;
    }
  }
  i2c_stop();
  return I2C_SUCCEEDED;
}

static uint8_t i2c_read_data(uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t ret = i2c_start((addr << 1) | 0x01);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  for (uint8_t i = 0; i < len; i++) {
    data[i] = i2c_read(i < (len - 1));
  }
  i2c_stop();
  return I2C_SUCCEEDED;
}

static inline uint8_t aht20_init(void) {
  uint8_t s = 0;
  uint8_t ret =
      i2c_write_data(AHT20_I2C_ADDR, (uint8_t[3]){0xBE, 0x08, 0x00}, 3);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ret = i2c_read_register(AHT20_I2C_ADDR, 0x71, &s);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  if (!(s & 0b100)) {
    return I2C_SUCCEEDED;
  }
  return ret;
}

static inline uint8_t aht20_get_status(uint8_t* s) {
  uint8_t ret = i2c_read_register(AHT20_I2C_ADDR, 0x71, s);
  return ret;
}

static inline uint8_t aht20_start(void) {
  uint8_t ret =
      i2c_write_data(AHT20_I2C_ADDR, (uint8_t[3]){0xAC, 0x33, 0x00}, 3);
  return ret;
}

static inline uint8_t aht20_read(uint32_t* temperature, uint32_t* humidity) {
  uint8_t data[7] = {0};
  uint8_t ret = i2c_read_data(AHT20_I2C_ADDR, data, 7);
  *humidity = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | (data[3]);
  *humidity >>= 4;
  *temperature =
      (((uint32_t)data[3] & 0xF) << 16) | ((uint32_t)data[4] << 8) | data[5];
  *temperature &= 0xFFFFF;
  return ret;
}

uint8_t bmp280_init(void) {
  // soft reset
  uint8_t ret = i2c_write_register(BMP280_I2C_ADDR, 0xE0, 0xB6);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }

  _delay_ms(2);
  // wait
  while (1) {
    uint8_t s = 0x00;
    ret = i2c_read_register(BMP280_I2C_ADDR, 0xF3, &s);
    if (ret != I2C_SUCCEEDED) {
      return ret;
    }
    if ((s & 1) == 0) {
      break;
    }
  }

  // read calibration data for temperature and pressure
  uint8_t i = 0;
  for (uint16_t cal_reg = 0x88; cal_reg <= 0x9e; cal_reg += 2) {
    ret = i2c_read_register16(BMP280_I2C_ADDR, cal_reg,
                              (uint16_t*)&bmp280_calibration_data[i++]);
    if (ret != I2C_SUCCEEDED) {
      return ret;
    }
  }

  ret =
      i2c_write_register(BMP280_I2C_ADDR, 0xF4, (3 << 5) | (3 << 2) | (0 << 0));
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }

  return I2C_SUCCEEDED;
}

uint8_t bmp280_start(void) {
  uint8_t ctrl = 0;
  uint8_t ret = i2c_read_register(BMP280_I2C_ADDR, 0xF4, &ctrl);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ctrl &= ~0b11;
  ctrl |= 1;  // forced mode (single measurement)
  ret = i2c_write_register(BMP280_I2C_ADDR, 0xF4, ctrl);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  return I2C_SUCCEEDED;
}

static inline int32_t bmp280_compensate_temperature(int32_t adc_temp,
                                                    int32_t* fine_temp) {
  int32_t var1, var2;

  var1 = ((((adc_temp >> 3) - ((int32_t)(uint16_t)dig_T1 << 1))) *
          (int32_t)dig_T2) >>
         11;
  var2 = (((((adc_temp >> 4) - (int32_t)(uint16_t)dig_T1) *
            ((adc_temp >> 4) - (int32_t)(uint16_t)dig_T1)) >>
           12) *
          (int32_t)dig_T3) >>
         14;

  *fine_temp = var1 + var2;
  return (*fine_temp * 5 + 128) >> 8;
}

static inline uint32_t bmp280_compensate_pressure(int32_t adc_press,
                                                  int32_t fine_temp) {
  int64_t var1, var2, p;

  var1 = (int64_t)fine_temp - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 =
      ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((int64_t)1 << 47) + var1) * ((int64_t)(uint16_t)dig_P1) >> 33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }

  p = 1048576 - adc_press;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dig_P8 * p) >> 19;

  p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
  return p;
}

static inline uint8_t bmp280_read(int32_t* temperature, int32_t* pressure) {
  uint8_t ret = i2c_start((BMP280_I2C_ADDR << 1) | 0x00);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }
  ret = i2c_write(0xF7);
  if (ret == 0) {
    return I2C_FAILED;
  }

  uint8_t data[6] = {0};
  ret = i2c_read_data(BMP280_I2C_ADDR, data, 6);
  if (ret != I2C_SUCCEEDED) {
    return ret;
  }

  int32_t adc_pressure =
      (uint32_t)data[0] << 12 | (uint32_t)data[1] << 4 | (uint32_t)data[2] >> 4;
  int32_t adc_temp =
      (uint32_t)data[3] << 12 | (uint32_t)data[4] << 4 | (uint32_t)data[5] >> 4;

  int32_t fine_temp;
  *temperature = bmp280_compensate_temperature(adc_temp, &fine_temp);
  *pressure = bmp280_compensate_pressure(adc_pressure, fine_temp);

  return I2C_SUCCEEDED;
}

static char tmp[16];
static void draw_graph(uint8_t buf_idx, int16_t const* const data_buf,
                       char const* const name, char const* const unit) {
  const int16_t last_avg =
      data_buf[buf_idx > 0 ? (uint8_t)(buf_idx - 1) : DATA_BUF_SIZE - 1];
  int16_t min_temp = last_avg;
  int16_t max_temp = last_avg;

  for (uint8_t i = 0; i < DATA_BUF_SIZE; i++) {
    const int16_t t = data_buf[(buf_idx + i) % DATA_BUF_SIZE];
    if (t > -1000) {
      if (t < min_temp) {
        min_temp = t;
      }
      if (t > max_temp) {
        max_temp = t;
      }
    }
  }

  uint16_t range = max_temp - min_temp;
  if ((range > 0) && (range < 10)) {
    range = 10;
    const int16_t c = (max_temp + min_temp) / 2;
    min_temp = c - 5;
    max_temp = c + 5;
  }

  uint8_t prev_x = 0;
  uint8_t prev_y = 0;

  uint8_t dc = 0;
  for (uint8_t i = 0; i < DATA_BUF_SIZE; i++) {
    uint16_t y = 32;
    const int16_t t = data_buf[(buf_idx + i) % DATA_BUF_SIZE];
    if (t > -1000) {
      if (range > 0) {
        y = (60 * (t - min_temp)) / range;
      }

      uint16_t x = (uint16_t)i * 128 / DATA_BUF_SIZE;
      if (dc) {
        u8g2_DrawLine(&u8g2, prev_x, 62 - prev_y, x, 62 - y);
        u8g2_DrawLine(&u8g2, prev_x, 62 - prev_y - 1, x, 62 - y - 1);
        u8g2_DrawLine(&u8g2, prev_x, 62 - prev_y + 1, x, 62 - y + 1);
      }
      // u8g2_DrawCircle(&u8g2, x, 62 - y, 1, U8G2_DRAW_ALL);
      prev_x = x;
      prev_y = y;
      dc++;
    }
  }

  u8g2_SetDrawColor(&u8g2, 2);
  for (int16_t t = (min_temp / 20); t <= (max_temp / 20); t++) {
    const int16_t tr = 20 * t;
    if (tr >= min_temp) {
      const uint8_t y = (60 * (tr - min_temp)) / range;
      for (uint8_t x = 0; x < 128; x += 4) {
        u8g2_DrawHLine(&u8g2, x, 62 - y, t == 0 ? 4 : 2);
      }
    }
  }
  u8g2_SetDrawColor(&u8g2, 1);

  u8g2_SetFont(&u8g2, BIG_LABEL_FONT);
  snprintf(tmp, sizeof(tmp), "%d.%d%s", max_temp / 10, abs(max_temp) % 10,
           unit);

  uint8_t w = u8g2_GetStrWidth(&u8g2, tmp);
  u8g2_SetDrawColor(&u8g2, 0);
  u8g2_DrawBox(&u8g2, 3, 3, w + 5, 15);
  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_DrawFrame(&u8g2, 3, 6, w + 5, 15);
  u8g2_DrawStr(&u8g2, 5, 18, tmp);

  snprintf(tmp, sizeof(tmp), "%d.%d%s", min_temp / 10, abs(min_temp) % 10,
           unit);

  w = u8g2_GetStrWidth(&u8g2, tmp);
  u8g2_SetDrawColor(&u8g2, 0);
  u8g2_DrawBox(&u8g2, 3, 49, w + 5, 15);
  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_DrawFrame(&u8g2, 3, 49, w + 5, 15);
  u8g2_DrawStr(&u8g2, 5, 61, tmp);

  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_SetFont(&u8g2, MEDIUM_LABEL_FONT);
  snprintf(tmp, sizeof(tmp), "%s", name);
  // w = u8g2_GetStrWidth(&u8g2, tmp);
  u8g2_DrawStr(&u8g2, 3, 6, tmp);
}

static void draw_value(int16_t value, char const* const name,
                       char const* const unit, bool bigunit, uint8_t offset) {
  u8g2_SetFont(&u8g2, main_font);
  const char sign = value < 0 ? '-' : ' ';
  const int16_t absv = abs(value);
  if ((bigunit) && (unit[0] != 'C')) {
    snprintf(tmp, sizeof(tmp), "%c%d.%d%s", sign, absv / 10, absv % 10, unit);
  } else {
    snprintf(tmp, sizeof(tmp), "%c%d.%d", sign, absv / 10, absv % 10);
  }
  uint8_t w = u8g2_GetStrWidth(&u8g2, tmp);
  u8g2_DrawStr(&u8g2, 100 - w + offset, 55, tmp);

  if ((bigunit) && (unit[0] == 'C')) {
    u8g2_DrawStr(&u8g2, 105, 55, unit);
    u8g2_DrawDisc(&u8g2, 103, 24, 5, U8G2_DRAW_ALL);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawCircle(&u8g2, 103, 24, 5, U8G2_DRAW_ALL);
    u8g2_DrawDisc(&u8g2, 103, 24, 2, U8G2_DRAW_ALL);
    u8g2_SetDrawColor(&u8g2, 1);
  }

  u8g2_SetFont(&u8g2, BIG_LABEL_FONT);
  if (!bigunit) {
    snprintf(tmp, sizeof(tmp), "%s (%s)", name, unit);
  } else {
    snprintf(tmp, sizeof(tmp), "%s", name);
  }
  u8g2_DrawStr(&u8g2, 3, 10, tmp);
}

static void i2c_error(void) {
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetDrawColor(&u8g2, 1);

  u8g2_SetFont(&u8g2, MEDIUM_LABEL_FONT);
  snprintf(tmp, sizeof(tmp), "I2C errors!");
  u8g2_DrawStr(&u8g2, 5, 18, tmp);
  u8g2_SendBuffer(&u8g2);
  while (1) {
    _delay_ms(1000);
  }
}

static int16_t ext_temp_buf[DATA_BUF_SIZE];
static int16_t hum_buf[DATA_BUF_SIZE];
static int16_t pres_buf[DATA_BUF_SIZE];

static inline void wait_for_events(void) {
  do {
    cli();
    while (!g_events) {
      sleep_enable();
      do {
        sei();
        sleep_cpu();
        sleep_disable();
      } while (false);
      wdt_reset();
      cli();
    }
    events = g_events;
    g_events = 0;
    sei();
  } while (0);
}

int main(void) {
  // 20MHz / 2 = 10MHz clock
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
  _PROTECTED_WRITE(CLKCTRL.OSC20MCTRLA, CLKCTRL_RUNSTDBY_bp);

  power_all_disable();
  configure_pins();
  configure_button();
  configure_led();
  configure_rtc();

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_avr_hw_i2c,
                                         u8x8_avr_delay);
  u8g2_SetI2CAddress(&u8g2, DISPLAY_I2C_ADDR >> 1);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetContrast(&u8g2, 255);
  // u8g2_SetFlipMode(&u8g2, 1);
  u8g2_SetPowerSave(&u8g2, 1);

  set_sleep_mode(SLEEP_MODE_STANDBY);

  configure_watchdog();

  _delay_ms(40);
  {
    uint8_t ret = bmp280_init();
    if (ret != I2C_SUCCEEDED) {
      i2c_error();
    }
    ret = aht20_init();
    if (ret != I2C_SUCCEEDED) {
      i2c_error();
    }
  }
  _delay_ms(10);

  uint16_t display_tmout = 0;
  int16_t ext_temp = read_ext_temp();
  int16_t int_temp = -1000;
  uint8_t avg_idx = 0;
  int32_t avg_ext_temp = 0;
  uint8_t buf_idx = 0;
  uint8_t disp_id = 0;

  for (uint8_t i = 0; i < DATA_BUF_SIZE; i++) {
    ext_temp_buf[i] = -1000;
    pres_buf[i] = -1000;
    hum_buf[i] = -1000;
  }

  g_events |= EV_BUTTON_PRESS;

  while (true) {
    wait_for_events();

    if (events & (EV_TEMP_TIMEOUT)) {
      PORTB.OUTCLR = RED_LED_PIN;
      ext_temp = read_ext_temp();

      avg_ext_temp += ext_temp;
      avg_idx++;
      if (avg_idx == 9) {
        aht20_start();
        bmp280_start();
      }
      if (avg_idx == 10) {
        avg_idx = 0;
        ext_temp_buf[buf_idx] = avg_ext_temp / 10;
        avg_ext_temp = 0;

        uint32_t humidity = 0;
        int32_t pressure = 0;
        {
          uint32_t temperature = 0;
          aht20_read(&temperature, &humidity);
          int_temp = (float)temperature / 1048576.0f * 2000.0f - 500.0;
        }
        {
          int32_t temperature = 0;
          bmp280_read(&temperature, &pressure);
        }
        hum_buf[buf_idx] = humidity / 1048576.0f * 1000.0f;
        pres_buf[buf_idx] = (pressure / 256) / 10;
        if (++buf_idx == DATA_BUF_SIZE) {
          buf_idx = 0;
        }
      }
      PORTB.OUTSET = RED_LED_PIN;
    }

    if (events & (EV_BUTTON_PRESS)) {
      u8g2_ClearBuffer(&u8g2);
      if (disp_id == 0) {
        const int16_t last_avg =
            ext_temp_buf[buf_idx > 0 ? (uint8_t)(buf_idx - 1)
                                     : DATA_BUF_SIZE - 1];

        if (last_avg > -1000) {
          draw_value(last_avg, "Ext. temp.", "C", true, 0);
        } else {
          draw_value(ext_temp, "Ext. temp.", "C", true, 0);
        }

        u8g2_SetPowerSave(&u8g2, 0);
        display_tmout = (3 * 1000) / 32;
      } else if (disp_id == 1) {
        draw_value(int_temp, "Int. temp.", "C", true, 0);
        display_tmout = (3 * 1000) / 32;
      } else if (disp_id == 2) {
        const int16_t last_pres =
            pres_buf[buf_idx > 0 ? (uint8_t)(buf_idx - 1) : DATA_BUF_SIZE - 1];
        draw_value(last_pres, "Pressure", "hPa", false, 30);
        display_tmout = (3 * 1000) / 32;
      } else if (disp_id == 3) {
        const int16_t last_hum =
            hum_buf[buf_idx > 0 ? (uint8_t)(buf_idx - 1) : DATA_BUF_SIZE - 1];
        draw_value(last_hum, "Humidity", "%", true, 20);
        display_tmout = (3 * 1000) / 32;
      } else if (disp_id == 4) {
        draw_graph(buf_idx, ext_temp_buf, "Temperature",
                   "\xb0"
                   "C");
        display_tmout = (10 * 1000) / 32;
      } else if (disp_id == 5) {
        draw_graph(buf_idx, hum_buf, "Humidity", "%");
        display_tmout = (10 * 1000) / 32;
      } else if (disp_id == 6) {
        draw_graph(buf_idx, pres_buf, "Pressure", "hPa");
        display_tmout = (10 * 1000) / 32;
      }
      u8g2_SendBuffer(&u8g2);
      disp_id++;
      if (disp_id == 7) {
        disp_id = 0;
      }
    }

    if ((display_tmout > 0) && (events & (EV_TICK))) {
      display_tmout--;
      if (display_tmout == 0) {
        disp_id = 0;
        u8g2_SetPowerSave(&u8g2, 1);
      }
    }
  }
}

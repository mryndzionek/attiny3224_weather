
#include <stdbool.h>
#include <util/delay.h>
#include <avr/io.h>

#include "u8x8_tinyavr_2.h"

#define P_CPU_NS (1000000000UL / F_CPU)

#define TWI_SCL_FREQ (400000UL)
#define TWI_T_RISE (300UL)

typedef enum
{
  TWI_INIT = 0,
  TWI_READY,
  TWI_ERROR
} TWI_Status;

static uint8_t RX_acked(void)
{
  return (!(TWI0.MSTATUS & TWI_RXACK_bm));
}

static void i2c_init(void)
{
  uint32_t baud = ((F_CPU / TWI_SCL_FREQ) - (((F_CPU * TWI_T_RISE) / 1000) / 1000) / 1000 - 10) / 2;
  TWI0.MBAUD = (uint8_t)baud;

  TWI0.MCTRLA = TWI_ENABLE_bm;
  TWI0.MADDR = 0x00;
  TWI0.MDATA = 0x00;

  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

static bool i2c_write(uint8_t data)
{
  TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
  TWI0.MDATA = data;
  while (!(TWI0.MSTATUS & TWI_WIF_bm))
    ;
  if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm))
    return false;
  return RX_acked();
}

static bool i2c_start(uint8_t addr)
{
  TWI0.MADDR = (addr << 1);
  while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)))
    ;
  if (TWI0.MSTATUS & TWI_ARBLOST_bm)
  {
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
      ;
    return false;
  }
  else if (TWI0.MSTATUS & TWI_RXACK_bm)
  {
    TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
      ;
    return false;
  }

  return true;
}

static void i2c_stop(void)
{
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;
  while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
    ;
}

uint8_t u8x8_byte_avr_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t *data;
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    data = (uint8_t *)arg_ptr;
    while (arg_int--)
      i2c_write(*data++);
    break;
  case U8X8_MSG_BYTE_INIT:
    i2c_init();
    break;
  case U8X8_MSG_BYTE_SET_DC:
    /* ignored for i2c */
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    i2c_start(u8x8_GetI2CAddress(u8x8));
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    i2c_stop();
    break;
  default:
    return 0;
  }
  return 1;
}

uint8_t u8x8_avr_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t cycles;
  (void) u8x8;
  (void) arg_ptr;

  switch (msg)
  {
  case U8X8_MSG_DELAY_NANO: // delay arg_int * 1 nano second
    // At 20Mhz, each cycle is 50ns, the call itself is slower.
    break;
  case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
// Approximate best case values...
#define CALL_CYCLES 26UL
#define CALC_CYCLES 4UL
#define RETURN_CYCLES 4UL
#define CYCLES_PER_LOOP 4UL

    cycles = (100UL * arg_int) / (P_CPU_NS * CYCLES_PER_LOOP);

    if (cycles > CALL_CYCLES + RETURN_CYCLES + CALC_CYCLES)
      break;

    __asm__ __volatile__(
        "1: sbiw %0,1"
        "\n\t" // 2 cycles
        "brne 1b"
        : "=w"(cycles)
        : "0"(cycles) // 2 cycles
    );
    break;
  case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
    while (arg_int--)
      _delay_us(10);
    break;
  case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
    while (arg_int--)
      _delay_ms(1);
    break;
  default:
    return 0;
  }

  return 1;
}

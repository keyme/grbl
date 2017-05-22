#include <stdint.h>

#include <avr/io.h>

#include "nuts_bolts.h"
#include "ad5121.h"
#include "spi.h"

#define AD_CMD_WRITE_RDAC     0x10
#define AD_CMD_RDAC_TO_EEPROM 0x70

#define AD_CMD_READ         0x30
#define AD_MASK_READ_RDAC   0x00
#define AD_MASK_READ_EEPROM 0x01

struct ad5121_dev {
  volatile uint8_t *cs_ddr;
  uint8_t cs_ddr_mask;
  volatile uint8_t *cs_port;
  uint8_t cs_pin;
};

static struct ad5121_dev devs[] = {
  {
    /* Gain Pot */
    .cs_ddr = &DDRC,
    .cs_ddr_mask = DDC0,
    .cs_port = &PORTC,
    .cs_pin = PC0,
  },
  {
    /* Offset Pot */
    .cs_ddr = &DDRC,
    .cs_ddr_mask = DDC1,
    .cs_port = &PORTC,
    .cs_pin = PC1,
  }
};

void ad5121_init(enum AD5121_ID dev_id)
{
  struct ad5121_dev dev = devs[dev_id];

  /* Set DDR of CS pin to output */
  *dev.cs_ddr |= 1 << dev.cs_ddr_mask;

  /* Deassert CS pin */
  *dev.cs_port |= 1 << dev.cs_pin;

}

void ad5121_write_pot(enum AD5121_ID dev_id, uint8_t val)
{
  struct ad5121_dev dev = devs[dev_id];
  uint8_t cmd[] = {AD_CMD_WRITE_RDAC, val};

  /* Assert CS pin */
  *dev.cs_port &= ~(1 << dev.cs_pin);

  spi_write(cmd, ARRAY_SIZE(cmd));

  /* Deassert CS pin */
  *dev.cs_port |= 1 << dev.cs_pin;

}

uint8_t ad5121_read_pot(enum AD5121_ID dev_id)
{
  struct ad5121_dev dev = devs[dev_id];
  uint8_t cmd[] = {AD_CMD_WRITE_RDAC, AD_MASK_READ_RDAC};
  uint8_t result = 0;

  /* Assert CS pin */
  *dev.cs_port &= ~(1 << dev.cs_pin);

  spi_write(cmd, ARRAY_SIZE(cmd));

  spi_transact_array(&result, &result, 1);

  /* Deassert CS pin */
  *dev.cs_port |= 1 << dev.cs_pin;

  return result;
}

void ad5121_store_pot(enum AD5121_ID dev_id)
{
  struct ad5121_dev dev = devs[dev_id];
  uint8_t cmd[] = {AD_CMD_RDAC_TO_EEPROM, 0x01};

  /* Assert CS pin */
  *dev.cs_port &= ~(1 << dev.cs_pin);

  spi_write(cmd, ARRAY_SIZE(cmd));

  /* Deassert CS pin */
  *dev.cs_port |= 1 << dev.cs_pin;
}

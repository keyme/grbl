#include "sram.h"
#include "spi.h"
#include "nuts_bolts.h"

/* Least and most significant bytes of x, of type uint16_t */
#define LSB(x) (x & 0x00FF)
#define MSB(x) ((x & 0xFF00) >> 8)

#define MODE_MASK 0xC0
#define MODE_IDX  6U

/* SPI insructions */
enum instruction_e {
  READ = 0x3,
  WRITE = 0x2,
  EDIO = 0x3B,
  EQIO = 0x38,
  RSTIO = 0xFF,
  RDMR = 0x5,
  WRMR = 0x1
};

void sram_init()
{
  /* Configure SCS pin as output */
  SCS_SRAM_DDR |= (1 << SCS_SRAM_DDR_PIN);

  /* Set SCS pin to high. The CS for this chip is active low */
  SCS_SRAM_PORT &= ~(1 << SCS_SRAM_PIN);

  /* SCK resting state is 0. Clock data on rising edge */
  spi_set_mode(0, 0);

  sram_set_mode(BYTE_MODE);
}

uint8_t _sram_mode_helper(uint8_t * data_out, uint8_t len)
{
  spi_set_mode(0, 0);

  uint8_t data_in[len];

  bit_false(SCS_SRAM_PORT, 1 << SCS_SRAM_PIN);
  spi_transact_array(data_out, data_in, len);
  bit_true(SCS_SRAM_PORT, 1 << SCS_SRAM_PIN);

  return (data_in[len - 1] & MODE_MASK) >> MODE_IDX;
}

uint8_t sram_read_mode()
{
  uint8_t data_out[2] = {RDMR, 0xFF};
  return _sram_mode_helper(data_out, 2);
}

void sram_set_mode(enum sram_mode_e mode)
{
  uint8_t data_out[2] = {WRMR, mode << MODE_IDX};
  _sram_mode_helper(data_out, 2);
}

uint8_t _sram_transact_helper(uint8_t * data_out, uint8_t len)
{
  /* Transacts the array data_out over SPI to the SRAM IC
     and returns the last byte received. */
  spi_set_mode(0, 0);

  uint8_t data_in[len];

  bit_false(SCS_SRAM_PORT, 1 << SCS_SRAM_PIN);
  spi_transact_array(data_out, data_in, len);
  bit_true(SCS_SRAM_PORT, 1 << SCS_SRAM_PIN);

  return data_in[len - 1];

}

uint8_t sram_read_byte(uint16_t addr)
{
  uint8_t data_out[4] = {READ, MSB(addr), LSB(addr), 0xFF};

  return _sram_transact_helper(data_out, 4);
}

void sram_write_byte(uint16_t addr, uint8_t val)
{
  /* Ignore the return value from _sram_transact_helper */
  uint8_t data_out[4] = {WRITE, MSB(addr), LSB(addr), val};

  _sram_transact_helper(data_out, 4);
}

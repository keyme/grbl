/*
  Not part of GRBL, KeyMe specific
*/

#include "spi.h"


void spi_set_mode(uint8_t cpol, uint8_t cpha)
{
  SPCR = ((1 << SPE)  | /* Enable */
          (0 << SPIE) | /* Disable SPI interrupt */
          (0 << DORD) | /* MSB first */
          (1 << MSTR) | /* Master mode */
          (1 << SPR1) | (0 << SPR0) | /* Set clock speed */
          (cpol << CPOL) | (cpha << CPHA)); /* Mode 0 */
}

void spi_init()
{

  /* TODO: Move these chip select initializations to
  their respective drivers. Since we don't have drivers yet,
  disable the CS lines here. */
  SCS_SRAM_DDR |= (1 << SCS_SRAM_DDR_PIN);
  SCS_SRAM_PORT |= (1 << SCS_SRAM_PIN);

  /* Configure MISO as input */
  SPI_DDR &= ~(1 << SPI_MISO);

  /* Pull-up on MISO */
  SPI_PORT |= (1 << SPI_MISO);

  /* MSB first */
  SPCR &= ~(1 << DORD);

  /* Configure MOSI, SCK as outputs */
  SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK);

  //Configure SCS Pins as outputs
  SCS_DDR_PORT |= SCS_DDR_MASK;

  //Set SCS to low for all steppers
  SCS_PORT &= ~(SCS_MASK);

  spi_set_mode(0, 0);

}

void spi_transact_array(uint8_t * dataout, uint8_t * datain, uint8_t len)
{

  uint8_t idx;

  for(idx = 0; idx < len; idx++) {
    if (dataout != NULL) {
      SPDR = dataout[idx];  //SPDR - SPI Data register
    } else {
      SPDR = 0;
    }

    uint16_t timeout = 0xFFFF;
    while ((SPSR & (1 << SPIF)) == 0) {
      if (timeout-- == 0) {
        return;
      } /* Wait for transmit to complete */
    }

    if (datain != NULL) {
      datain[idx] = SPDR;
    }
  }

}

void spi_read(uint8_t * datain, uint8_t len) 
{
  spi_transact_array(NULL, datain, len);
}

void spi_write(uint8_t * dataout, uint8_t len)
{
  spi_transact_array(dataout, NULL, len);
}


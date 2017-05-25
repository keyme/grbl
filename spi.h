/*
  Not part of GRBL, KeyMe specific
*/

#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>
#include "system.h"


#define SPI_PORT        PORTB
#define SPI_DDR         DDRB
#define DD_MISO         DDB3
#define DD_MOSI         DDB2
#define DD_SCK          DDB1
#define DD_SS           DDB0

#define C_SPIE 0        //SPI Interrupt Enable
#define C_SPE  1        //SPI Enable
#define C_DORD 0        //SPI Data Order; 1 - LSB of word transmitted first; 0 - MSB first
#define C_MSTR 1        //1 - Master SPI mode; 0 - Slave SPI mode
#define C_CPOL 0        //Clock polarity; 1 - SCK is high when IDLE; 0 - low
#define C_CPHA 0        //1 - Leading edge setup, Trailing edge sample; 0 - vice versa
#define C_SPR1 0        //Control SCK rate - selected as TODO: configure SPI Clock Rate
#define C_SPR0 1

void spi_init();
void spi_reconfigure(uint8_t cpol, uint8_t cpha);

// Transmit an array of bytes and receive a byte for each byte transmitted
void spi_transact_array(uint8_t * dataout, uint8_t * datain, uint8_t len);

// Wrapper function for spi_transact_array, but no data is sent
void spi_read(uint8_t * datain, uint8_t len);

// Wrapper function for spi_transact_array, but not data is expected back
void spi_write(uint8_t * dataout, uint8_t len);
#endif //H_

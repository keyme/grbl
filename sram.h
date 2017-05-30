#ifndef SRAM_H
#define SRAM_H

#include "system.h"

enum sram_mode_e {
  BYTE_MODE = 0,
  SEQ_MODE = 1U,
  PAGE_MODE = 2U

};

void sram_init();

uint8_t sram_read_byte(uint16_t addr);

void sram_write_byte(uint16_t addr, uint8_t val);

uint8_t sram_read_mode();

void sram_set_mode(enum sram_mode_e mode);

#endif

#ifndef _AD5121_H_
#define _AD5121_H_

#include <stdint.h>

enum AD5121_ID {
  AD5121_0 = 0,
  AD5121_1 = 1
};

void ad5121_init(enum AD5121_ID dev_id);
void ad5121_write_pot(enum AD5121_ID dev_id, uint8_t val);
uint8_t ad5121_read_pot(enum AD5121_ID dev_id);
void ad5121_store_pot(enum AD5121_ID dev_id);

#endif

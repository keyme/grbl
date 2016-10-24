/*
  Not part of Grbl. KeyMe specific
*/

#ifndef adc_h
#define adc_h

void adc_init();

uint16_t adc_read_channel(uint8_t channel);

#endif

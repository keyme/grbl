/*
  Not part of Grbl. KeyMe specific.
  
  Signal is one layer of abstraction above adc.h and adc.c.

  ADC values are read in specific time intervals, filtered and
  stored in the appropriate arrays.
*/

#ifndef signals_h
#define signals_h

#include "system.h"

#define FORCE_VALUE_INDEX 4  // Index of force value in signals.adc_samples
#define REV_VALUE_INDEX 5  // Index of revision value in signals.adc_samples

// Alias the force reading since it is often used throughout the code
#define FORCE_VAL signals.adc_samples[FORCE_VALUE_INDEX]

typedef struct {
  uint8_t pause;  // Pause the reading of ADC values periodically with a callback
  uint16_t adc_samples[VOLTAGE_SENSOR_COUNT];  // Filtered ADC readings
  uint16_t callback_period;  // Period between ADC readings TODO: Add command to change this over serial  
} signals_t;
signals_t signals;

void signals_init();
void signals_update_revision(); // Called from main.c
void signals_update_motors();
void signals_update_force();
void signals_callback(); // Callback first registered in main.c

#endif


/*
  Not part of Grbl. KeyMe specific.
  
  Signal is one layer of abstraction above adc.h and adc.c.

  ADC values are read in specific time intervals, filtered and
  stored in the appropriate arrays.
*/

#ifndef signals_h
#define signals_h

#include "system.h"

#define FORCE_VALUE_INDEX 4  // Index of force value in analog_voltage_readings
#define REV_VALUE_INDEX 5  // Index of revision value in analog_voltage_readings

// Array of latest ADC readings
uint16_t analog_voltage_readings[VOLTAGE_SENSOR_COUNT];  // Filtered ADC readings

void signals_update_revision(); // Called from main.c
void signals_callback();        // Callback first registered in main.c

#endif


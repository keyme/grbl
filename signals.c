/*
  Not part of Grbl. KeyMe specific.

  Signal is one layer of abstraction above adc.h and adc.c.

  ADC values are read in specific time intervals, filtered and
  stored in the appropriate arrays.
*/

#include "signals.h"
#include "adc.h"
#include "systick.h"

#define N_FILTER 3
#define SIGNALS_DEFAULT_INTERVAL 100
#define X_BUF(i) adc_samples_x[FORCE_VALUE_INDEX][i]

// Unfiltered ADC readings
int16_t adc_samples_x[VOLTAGE_SENSOR_COUNT][N_FILTER];

void signals_init()
{
  signals.pause = 0;
  signals.callback_period = SIGNALS_DEFAULT_INTERVAL;
}

// Update motors ADC readings
void signals_update_motors()
{
  uint8_t idx;

  for (idx = 0; idx < N_AXIS; idx++) {
    // Assumes the motors are on ADC channels 0-3 and in the same
    // order in signals.adc_samples. If the pins are changed, a
    // motors should be mapped to ADC channels.
    signals.adc_samples[idx] = raw_adc_readings[idx];
  }
}

// Filter and update force ADC reading
void signals_update_force()
{
  // Since the raw adc value changes based on an interrupt,
  // copy the raw value at the beginning of this function to
  // ensure that the value being used doesn't change during 
  // during the function call.
  uint16_t raw_value = raw_adc_readings[FORCE];

  // Index in signal buffer
  static const uint8_t n = N_FILTER - 1;

  X_BUF(n) = raw_value;

  /*
    Filter:
      Moving average Hanning Filter:
      y[k] = 0.25 * (x[k] + 2x[k-1] + x[k-2])
  */
  signals.adc_samples[FORCE_VALUE_INDEX] = (uint16_t)(
  (X_BUF(n) + (X_BUF(n - 1) << 1) + X_BUF(n - 2)) >> 2);

  // Advance all values in the unfiltered array
  memmove(&X_BUF(0), &X_BUF(1) ,sizeof(uint16_t) * N_FILTER - 1);

}

void signals_callback()
{
  signals_update_motors();
  signals_update_revision();

  if (signals.pause) {
    // signals.pause needs to be set before the force servoing cycle
    // is started to prevent the signals_callback from asynchronously
    // updating the force value while force servoing.
    systick_register_callback(signals.callback_period, signals_callback);
    return;
  }  

  signals_update_force();
 
  // Register callback to this function in SIGNALS_CALLBACK_INTERVAL milliseconds
  systick_register_callback(signals.callback_period, signals_callback);
}

// Read value from revision voltage divider
// No nead to filter since the value is constant.
// Only needs to be called once during initialization
void signals_update_revision()
{
  signals.adc_samples[REV_VALUE_INDEX] = raw_adc_readings[REV];
}


/*
  Not part of Grbl. KeyMe specific.

  System tick implementation. Global sys_tick value is updated.
  Callbacks can be registed to be called after a certain time.

  sys_tick uses Timer1. Timer1 should not be used anywhere else.

*/
#include "systick.h"
#include "nuts_bolts.h"

#define MAX_CALLBACKS 32

callback_t systick_callbacks_array[MAX_CALLBACKS];
uint8_t systick_len;  // Length of callback array 

// Helper function for qsort. Used in systick_sort_callback_array
int cmp(const void * a, const void * b)
{
  callback_t *cb_a = (callback_t *)a;
  callback_t *cb_b = (callback_t *)b;
  
  return (cb_a->callback_time - cb_b->callback_time);

}

// Sort the callback array according to lowest callback time
void systick_sort_callback_array()
{
  qsort(systick_callbacks_array, systick_len, sizeof(callback_t), cmp); 
}

void systick_service_callbacks()
{
  if (!systick_len) {
    return;
  }

  // Sort the callback array
  systick_sort_callback_array();
  
  // Count the number of expired callbacks
  uint8_t idx = 0;
  while (systick_callbacks_array[idx].callback_time <= sys_tick) {
    idx++;
  }
  
  while (idx > 0) {
    // Call the callback function
    systick_callbacks_array[0].cb_function();

    // Shift all entries in the callback array to the left and decrement the index
    memmove(&systick_callbacks_array[0], &systick_callbacks_array[1], sizeof(callback_t) * (--systick_len));
    idx--;
  }
}

void systick_init()
{
  // Reset sys_tick
  sys_tick = 0;

  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64x - 16 MHz / 64 = 250 kHz

  // CTC mode - Clear Timer on Compare
  bit_true(TCCR1B, 1 << WGM12);
  
  // Inialize the counter
  TCNT1 = 0;
  
  // Initialize timer compare value
  OCR1A = 250; // 250 kHz/250 = 1 kHz, hence, sys_tick will tick every ms 
  
  // Enable the compare interrupt in Timer Interrupt Mask Register
  bit_true(TIMSK1, 1 << OCIE1A);

  // Initialize length of the callback array
  systick_len = 0; 

}

// Every millisecond
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;
  sys_tick++;
}

// Schedule a callback
void systick_register_callback(uint32_t ms_later, void (*func)())
{
 
  uint64_t callback_time = sys_tick + ms_later;

  callback_t to_insert;
  to_insert.callback_time = callback_time;
  to_insert.cb_function = func;

  // Insert callback into callbacks array
  systick_callbacks_array[systick_len] = to_insert;

  // Increase the length of the array
  systick_len++; 

}

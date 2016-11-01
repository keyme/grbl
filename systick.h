/*
  Not part of Grbl. KeyMe specific.
 
  System tick implementation. Global sys_tick value is updated.
  Callbacks can be registed to be called after a certain time.
  
  sys_tick uses Timer1. Timer1 should not be used anywhere else.
 
*/

#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "system.h"

typedef struct {
  uint64_t callback_time;
  void (*cb_function)();
} callback_t;

uint64_t sys_tick;

void systick_init();

void systick_register_callback(uint32_t, void(*)());

void systick_service_callbacks();

#endif

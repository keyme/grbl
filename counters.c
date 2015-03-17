/*
  counters.c - code pertaining to encoders and other counting methods
  Part of Grbl

  Copyright (c) 2014 Adam Shelly

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include "system.h"
#include "counters.h"

uint32_t alignment_debounce_timer=0; 
#define PROBE_DEBOUNCE_DELAY_MS 25


counters_t counters = {{0}};

// Counters pin initialization routine.
void counters_init() 
{
  //encoders and feedback //TODO: move to new file
  FDBK_DDR &= ~(FDBK_MASK); // Configure as input pins
  FDBK_PORT |= FDBK_MASK;   // Enable internal pull-up resistors. Normal high operation. 
  counters.state = FDBK_PIN&FDBK_MASK; //record initial state

  counters_enable(0); //default to no encoder
}


void counters_enable(int enable) 
{
  if (enable) {
    FDBK_PCMSK |= FDBK_MASK;    // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << FDBK_INT);   // Enable Pin Change Interrupt
  }
  else {
    FDBK_PCMSK &= ~FDBK_MASK;    // Disable specific pins of the Pin Change Interrupt
    PCICR &= ~(1 << FDBK_INT);   // Disable Pin Change Interrupt
  }

}


// Resets the counts for an axis
void  counters_reset(uint8_t axis)
{
  counters.counts[axis]=0;
  if (axis == Z_AXIS) { counters.idx=0; }
}


// Returns the counters pin state. Triggered = true.  and counters state monitor.
count_t counters_get_count(uint8_t axis)
{
  return counters.counts[axis];
}


int debounce(uint32_t* bounce_clock, int16_t lockout_ms) {
  uint32_t clock = masterclock;
  //allow another reading if lockout has expired 
  if ( (uint32_t)(clock - *bounce_clock) >= lockout_ms ) {
    *bounce_clock = clock;
    return 1;
  }
  return 0;
}


ISR(FDBK_INT_vect) {
  uint8_t state =  FDBK_PIN&FDBK_MASK;
  uint8_t change = (state^counters.state);
  int8_t dir=0;

  //look for encoder change
  if (change & ((1<<Z_ENC_CHA_BIT)|(1<<Z_ENC_CHB_BIT))) { //if a or b changed
    counters.anew = (state>>Z_ENC_CHA_BIT)&1;
    dir = counters.anew^counters.bold ? 1 : -1;
    counters.bold = (state>>Z_ENC_CHB_BIT)&1;
    counters.counts[Z_AXIS] += dir;
  }

  //count encoder indexes
  if (change & (1<<Z_ENC_IDX_BIT)) { //idx changed
      uint8_t idx_on = ((state>>Z_ENC_IDX_BIT)&1);
      if (idx_on) {
        counters.idx += dir;
      }
  }

  //count conveyor axis alignment pulses.
  if (change & (1<<ALIGN_SENSE_BIT)) { //sensor changed
    if (debounce(&alignment_debounce_timer, PROBE_DEBOUNCE_DELAY_MS)){
      if (!(state&PROBE_MASK)) { //low is on.
        counters.counts[C_AXIS]++;
      }
    }
  }
  counters.state = state;
}

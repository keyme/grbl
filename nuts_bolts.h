/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
  Part of Grbl

  Copyright (c) 2011-2014 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef nuts_bolts_h
#define nuts_bolts_h

#define false 0
#define true 1

enum e_axis {
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS,
  C_AXIS,
  N_AXIS
};

#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define TICKS_PER_MICROSECOND (F_CPU/1000000)

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define bit(n) (1 << (n))
#define bit_true(x,mask) ((x) |= (mask))
#define bit_false(x,mask) ((x) &= ~(mask))
#define bit_toggle(x,mask) ((x) ^= (mask))
#define bit_istrue(x,mask) (((x) & (mask)) != 0)
#define bit_isfalse(x,mask) (((x) & (mask)) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

//convert index into bitmask. using macros instead of functions to take care of guaranteed layout
#define get_direction_mask(i)  ((1<<(X_DIRECTION_BIT))<<(i))
#define get_step_mask(i)       ((1<<(X_STEP_BIT))<<(i))

uint8_t get_axis_idx(char axis_letter);

float hypot_f(float x, float y);

#endif

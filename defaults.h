/*
  defaults.h - defaults settings configuration file
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon

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

/* The defaults.h file serves as a central default settings file for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   here are supplied by users, so your results may vary. However, this should give you
   a good starting point as you get to know your machine and tweak the settings for your
   our nefarious needs. */

#ifndef defaults_h
#define defaults_h

#ifdef DEFAULTS_GENERIC
  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 250.0
  #define DEFAULT_Y_STEPS_PER_MM 250.0
  #define DEFAULT_Z_STEPS_PER_MM 250.0
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_DECIMAL_PLACES 3
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
  #define DEFAULT_SINGLE_STEP_RATE 10  // steps/sec
#endif


#ifdef DEFAULTS_KEYME_FM
  // generic default for KeyMe Machine
  #define DEFAULT_X_STEPS_PER_MM 66.666
  #define DEFAULT_Y_STEPS_PER_MM 40.0
  #define DEFAULT_Z_STEPS_PER_MM 400.0
  #define DEFAULT_C_STEPS_PER_MM 58.0
  #define DEFAULT_X_MAX_RATE 1320.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 2100.0 //
  #define DEFAULT_Z_MAX_RATE 400.0 //
  #define DEFAULT_C_MAX_RATE 300.0 //
  #define DEFAULT_X_ACCELERATION (50.0*60*60) // mm/sec^2
  #define DEFAULT_Y_ACCELERATION (75.0*60*60) // mm/sec^2
  #define DEFAULT_Z_ACCELERATION (8.0*60*60) //  mm/sec^2
  #define DEFAULT_C_ACCELERATION (4.50*60*60) // mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 100.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 50.0 // mm
  #define DEFAULT_C_MAX_TRAVEL 762.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 15
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 64
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 250   // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_DECIMAL_PLACES 3
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 0 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 1 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 1  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir (negative with keyme mods)
  #define DEFAULT_HOMING_FEED_RATE 100.0 // mm/min
  #define DEFAULT_X_HOMING_SEEK_RATE 200.0 // mm/min
  #define DEFAULT_Y_HOMING_SEEK_RATE 200.0 // mm/min
  #define DEFAULT_Z_HOMING_SEEK_RATE 200.0 // mm/min
  #define DEFAULT_C_HOMING_SEEK_RATE 200.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
  #define DEFAULT_SINGLE_STEP_RATE 10  // steps/sec
  #define DEFAULT_MICROSTEPPING 0x55   //half stepping on all axes
  #define DEFAULT_DECAY_MODE 0         //slowest
  #define DEFAULT_COUNTS_PER_IDX 4000  //counts per encoder rev
  #define DEFAULT_FORCE_SENSOR_LEVEL 77 // 1.5V out of 5.0V=255 max
  #define DEFAULT_MAG_GAP_LIMIT 12.5 //in mm (units)
  #define DEFAULT_MAG_GAP_ENABLED false
  #define DEFAULT_USE_LOAD_CELL false
#endif

#ifdef DEFAULTS_BENCH
  // generic default for KeyMe development benchtop
  #define DEFAULT_X_STEPS_PER_MM 200.0
  #define DEFAULT_Y_STEPS_PER_MM 200.0
  #define DEFAULT_Z_STEPS_PER_MM 200.0
  #define DEFAULT_C_STEPS_PER_MM 200.0
  #define DEFAULT_X_MAX_RATE 600.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 600.0 //
  #define DEFAULT_Z_MAX_RATE 600.0 //
  #define DEFAULT_C_MAX_RATE 600.0 //
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2
  #define DEFAULT_C_ACCELERATION (6.0*60*60) // 1*60*60 mm/min^2
  #define DEFAULT_X_MAX_TRAVEL 250.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 250.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 250.0 // mm
  #define DEFAULT_C_MAX_TRAVEL 250.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 15
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_DECIMAL_PLACES 3
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 1  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 10.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 20.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
  #define DEFAULT_SINGLE_STEP_RATE 10  // steps/sec
  #define DEFAULT_MICROSTEPPING 0x55   //half stepping on all axes
  #define DEFAULT_DECAY_MODE 0         //slowest
  #define DEFAULT_COUNTS_PER_IDX 4000  //counts per encoder rev
#endif

#endif

/*
  settings.h - eeprom configuration handling
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

#ifndef settings_h
#define settings_h

#include "system.h"


// This is a bit of compiler magic to ensure that we don't
// accidentally optimize away variables that are only used by external
// tools (version numbers, ident blocks, etc)
#define ALWAYS_KEEP(v) __asm__ __volatile__("" :: "m" (v))

// This is a preprocessor trick. STR simply token pastes the value
// given to it. By wrapping a defined value in XSTR, it forces the
// evaluation of the token before pasting. So if you defined 'VERSION'
// via -DVERSION=foo, XSTR(VERSION) == "foo", and STR(VERSION) == "VERSION"
#define STR(s) #s
#define XSTR(s) STR(s)

#define GRBL_VERSION_BUILD __DATE__ " " __TIME__

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 73

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_AUTO_START         bit(1)
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL 1
#define EEPROM_ADDR_PARAMETERS 512
#define EEPROM_ADDR_STARTUP_BLOCK 768
#define EEPROM_ADDR_BUILD_INFO 992

// Define EEPROM address indexing for coordinate parameters
//@TODO: can reduce this for EEPROM space.
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)



// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;  // If max value 255, steppers do not disable.
  float junction_deviation;
  float arc_tolerance;
  uint8_t flags;  // Contains default boolean settings
  uint8_t homing_dir_mask;
  float homing_feed_rate;  //slow resolve sensor
  float homing_seek_rate[N_AXIS]; //seek to sensor
  uint16_t homing_debounce_delay;
  float homing_pulloff ;
  uint8_t microsteps;  //2 bits per motor
  uint8_t decay_mode;  //0..3  slow-->fast
  uint8_t force_sensor_level;  //0..255  low-->high sensitivity
  float mag_gap_limit;  // Maximum gap between two magazines at which point an alarm is thrown
  uint8_t mag_gap_enabled;  //If 0, then do not check the gap between magazines
  uint8_t use_load_cell;  // 0 - no load cell, 1 - load cell
  uint8_t lc_daughter_card; // 0 - digital pots, 1 - daughter card
  uint8_t use_spi; // 0 - no, 1 - yes
  uint8_t spi_motor_drivers; // 0 - no, 1 - yes
  uint8_t x_microsteps;
  uint8_t y_microsteps;
  uint8_t z_microsteps;
  uint8_t c_microsteps;
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// A helper method to set new settings from command line
uint8_t settings_store_global_setting(int parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

void settings_store_build_info(char *line);

uint8_t settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

#endif

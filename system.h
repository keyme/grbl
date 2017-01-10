/*
  system.h - Header for system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014 Sungeun K. Jeon

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

#ifndef system_h
#define system_h

// Define system header files and standard libraries used by Grbl
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define Grbl configuration and shared header files
#include "config.h"
#include "defaults.h"
#include "cpu_map.h"
#include "nuts_bolts.h"

// Define system executor bit map. Used internally by runtime protocol as runtime command flags,
// which notifies the main program to execute the specified runtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the runtime protocol only needs to check for a non-zero value to
// know when there is a runtime command to execute.
#define EXEC_RUNTIME_REPORT bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_ALARM          bit(5) // bitmask 00100000
#define EXEC_CRIT_EVENT     bit(6) // bitmask 01000000

#define REQUEST_STATUS_REPORT  bit(0)
#define REQUEST_LIMIT_REPORT   bit(1)
#define REQUEST_COUNTER_REPORT bit(2)
#define REQUEST_VOLTAGE_REPORT bit(3)
#define REQUEST_EDGE_REPORT    bit(4)

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE       0      // Must be zero. No flags.
#define STATE_ALARM      bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING     bit(2) // Performing homing cycle
#define STATE_QUEUED     bit(3) // Indicates buffered blocks, awaiting cycle start.
#define STATE_CYCLE      bit(4) // Cycle is running
#define STATE_HOLD       bit(5) // Executing feed hold
#define STATE_FORCESERVO bit(6) // Force servo process
#define STATE_HOME_ADJUST bit(7) // Update the minimum homing rate when some axes complete homing cycle
#define STATE_PROBING    bit(8)

// Define Grbl alarm codes. Listed most to least serious
#define ALARM_SOFT_LIMIT  bit(0) // soft limits exceeded
#define ALARM_HARD_LIMIT  bit(1) // hard limit hit
#define ALARM_ABORT_CYCLE bit(2) // abort during motion
#define ALARM_PROBE_FAIL  bit(3) // probe not found
#define ALARM_HOME_FAIL   bit(4) // home switch transition not found
#define ALARM_ESTOP       bit(5) // external estop pressed
#define ALARM_FORCESERVO_FAIL bit(6) // force value not reached while servoing
#define ALARM_MAG_MISSING  bit(7) //Mag expected but not sensed

// Define system flags
#define SYSFLAG_EOL_REPORT bit(0)  // Block is done executing, report linenum
#define SYSFLAG_AUTOSTART  bit(1)  // autostart is active

// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint16_t state;                 // Tracks the current state of Grbl.
  uint8_t flags;                 // see SYSFLAG_xxx above
  uint8_t alarm;                 // see ALARM_xxx above. which alarm(s) are active
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
                                 // NOTE: This may need to be a volatile variable, if problems arise.
  int32_t probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
  uint8_t lock_mask;             // Mask which determines the state of axis 'locking' (aka braking)
} system_t;
extern system_t sys;

typedef struct {
  volatile uint8_t execute;      // Global system runtime executor bitflag variable. See EXEC bitmasks.
  volatile uint8_t limits;                 //limit
  volatile uint8_t report_rqsts;   //requestsd reports
} sys_flags_t;
extern volatile sys_flags_t sysflags;


#ifndef UNPROTECTED_VOLATILES
  #define SYS_EXEC GPIOR0
#else
  #define SYS_EXEC sysflags.execute
#endif

extern uint32_t masterclock;   //long running clock w/ 1ms resolution. rolls over every 49.7 days

// Initialize the serial protocol
void system_init();

// Executes an internal system command, defined as a string starting with a '$'
uint8_t system_execute_line(char *line);

// Checks and executes a runtime command at various stop points in main program
void system_execute_runtime();

// Execute the startup script lines stored in EEPROM upon initialization
void system_execute_startup(char *line);

//  * Utilities for line numbeirng *
// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So grbl increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.

// But We don't need such a big value, and we do need the high bit free
//#define MAX_LINE_NUMBER 9999999
#define LINENUMBER_EMPTY_BLOCK 0x8000 //the other bit, used as a flag
#define LINENUMBER_SPECIAL      0x4000 //denotes Homing and Probing among a couple other processes
#define LINENUMBER_SPECIAL_SERVO 0x10000 //denotes force servoing
#define LINENUMBER_MAX         (LINENUMBER_SPECIAL-1)
#define LINEMASK_OFF_EDGE         (0x0)
#define LINEMASK_ON_EDGE         (0x1)
#define LINEMASK_DONE           (0x2)
typedef uint32_t linenumber_t;


void linenumber_init();
uint8_t linenumber_insert(linenumber_t line_number);
linenumber_t linenumber_get();
linenumber_t linenumber_peek();


enum {
  time_STEP_ISR,
  time_HOMING,
  time_PROBE,
  time_CLOCK
};

#define ACTIVE_TIMER time_STEP_ISR
#define TIME_OFF(tid)  (((tid)==ACTIVE_TIMER)?(TIMING_PORT|=TIMING_MASK):(void)tid)
#define TIME_ON(tid)  (((tid)==ACTIVE_TIMER)?(TIMING_PORT&=~TIMING_MASK):(void)tid)
#define TIME_TOGGLE(tid)  (((tid)==ACTIVE_TIMER)?(TIMING_PIN|=TIMING_MASK):(void)tid)

// Voltage Monitoring
#define VOLTAGE_SENSOR_COUNT 6 // number of devices (X,Y,Z,C,F,RD) for which voltage is measured
// Reasoning for commenting this function is in system.c.
//void init_ADC();

// Helper for squelching 'unused parameter' errors
#define UNUSED(param) (void)(param);


#endif

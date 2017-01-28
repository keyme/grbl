/*
  protocol.c - controls Grbl execution protocol and procedures
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

#include "system.h"
#include "serial.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "motion_control.h"
#include "report.h"
#include "progman.h"
#include "systick.h"
#include "magazine.h"

#define STATUS_REPORT_RATE_MS 333  //3 Hz

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

static uint32_t report_clock=0; //time until next automatic report
static uint8_t next_report=REQUEST_STATUS_REPORT;

// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
static void protocol_execute_line(char *line)
{
  protocol_execute_runtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort

  uint8_t status = STATUS_OK;

  if (line[0] == '$') {
    // Grbl '$' system command
    status = system_execute_line(line);
  } else if (sys.state == STATE_ALARM) {
    // Everything else is gcode. Block if in alarm mode.
    status = STATUS_ALARM_LOCK;
    report_status_message(STATUS_ALARM_LOCK);

  } else if (line[0] == CMD_LINE_START) {
    // This is a special start command which is guaranteed to be sequenced after
    // the previous line - it won't be picked out of the serial stream while
    // gc_execute_line is still parsing the previous line.
    SYS_EXEC |= EXEC_CYCLE_START;
  } else {
    status = gc_execute_line(line);
  }

  /* If there was an error, report it */
  if (status) {
    report_status_message(status);
  }
}


/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // ------------------------------------------------------------
  // Complete initialization procedures upon a power-up or reset.
  // ------------------------------------------------------------

  // Print welcome message
  report_init_message();

  // Check for and report alarm state after a reset, error, or an initial power up.
  if (sys.state == STATE_ALARM) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
  } else {
    // All systems go!
    sys.state = STATE_IDLE; // Set system to ready. Clear all state flags.
  }

  // ---------------------------------------------------------------------------------
  // Primary loop! Upon a system abort, this exits back to main() to reset the system.
  // ---------------------------------------------------------------------------------

  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {
    // Process one line of incoming Gcode data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.

    // NOTE: The Program Manager handles the filtering of comments,
    // spaces, and block skips. All data *should* come through as
    // complete newline delimited blocks

    while(progman_read(&c)) {
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        protocol_execute_line(line); // Line is complete. Execute it!
        char_counter = 0;
      } else {
        if (char_counter >= LINE_BUFFER_SIZE-1) {
        // Detect line buffer overflow. Report error and reset line buffer.
        report_status_message(STATUS_OVERFLOW);
            char_counter = 0;
        } else {
        /* Put the capitalized letter in the buffer */
        line[char_counter++] = (c >= 'a' && c <= 'z') ? (c - 'a' + 'A') : c;
        }
      }
    }

    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_runtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.

  }
  return; /* Never reached */
}


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sysflags.execute variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_runtime()
{
  uint8_t rt_exec = SYS_EXEC; // Copy to avoid calling volatile multiple times
  uint32_t clock = masterclock;

  // Give the program manager some time to manage the serial traffic
  progman_execute();

  // Service SysTick Callbacks
  systick_service_callbacks();
  
  if (clock >= (report_clock +  STATUS_REPORT_RATE_MS) || clock < report_clock) {
    rt_exec|= EXEC_RUNTIME_REPORT;
    sysflags.report_rqsts|=next_report;
    next_report^=(REQUEST_STATUS_REPORT|REQUEST_LIMIT_REPORT); //toggle between these two
    report_clock = clock;
  }
  st_check_disable();


  if (rt_exec) { // Enter only if any bit flag is true

    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical events. Error events identified by sys.alarm flags
      report_alarm_message(sys.alarm);
        // check critical
      if (rt_exec & EXEC_CRIT_EVENT) {
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false(SYS_EXEC,EXEC_RESET); // Disable any existing reset
        do {
          // Block EVERYTHING until user issues reset or power cycles
          // (requires the program manager to handle from the incoming
          // serial stream). Hard limits typically occur while
          // unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the
          // incoming stream. The same could be said about soft
          // limits. While the position is not lost, the incoming
          // stream could be still engaged and cause a serious crash
          // if it continues afterwards.
          progman_execute();
        } while (bit_isfalse(SYS_EXEC,EXEC_RESET));
      }
      bit_false(SYS_EXEC,(EXEC_ALARM | EXEC_CRIT_EVENT));
      sys.alarm = 0; //clear reported alarms
    }

    // Execute system abort.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }

    // Execute and serial print status
    if (rt_exec & EXEC_RUNTIME_REPORT) {
      uint8_t reports=sysflags.report_rqsts;
      sysflags.report_rqsts=0;   //TODO: potential race here if isr requests another right after we read it.

      if (reports & REQUEST_STATUS_REPORT) {
        if (!report_realtime_status()){ //ensure no more lines to report
          reports &= ~REQUEST_STATUS_REPORT;
        }
      }
      //elsif forces 1 report per loop max
      else if (reports & REQUEST_LIMIT_REPORT) {
        report_limit_pins();
        reports &= ~REQUEST_LIMIT_REPORT;
      }
      else if (reports & REQUEST_COUNTER_REPORT) {
        report_counters();
        reports &= ~REQUEST_COUNTER_REPORT;
      }
      else if (reports & REQUEST_VOLTAGE_REPORT) {
        report_voltage();
        reports &= ~REQUEST_VOLTAGE_REPORT;
      }
      else if (reports & REQUEST_EDGE_REPORT) {
        magazine_report_edge_events();
        reports &= ~REQUEST_EDGE_REPORT;
      }
      if (0==(sysflags.report_rqsts|=reports)) { //if all reports done and no new requests, clear report flag
        bit_false(SYS_EXEC,EXEC_RUNTIME_REPORT);
      }
    }

    // Execute a feed hold with deceleration, only during cycle.
    if (rt_exec & EXEC_FEED_HOLD) {
      // !!! During a cycle, the segment buffer has just been reloaded and full. So the math involved
      // with the feed hold should be fine for most, if not all, operational scenarios.
      if (sys.state == STATE_CYCLE) {
        sys.state = STATE_HOLD;
        st_update_plan_block_parameters();
        st_prep_buffer();
        sys.flags &=~ SYSFLAG_AUTOSTART; // Disable planner auto start upon feed hold.
      }
      bit_false(SYS_EXEC,EXEC_FEED_HOLD);
    }

    // Execute a cycle start by starting the stepper interrupt begin executing the blocks in queue.
    // block Start while homing/force-servoing.
    if ((rt_exec & EXEC_CYCLE_START) && !(sys.state & (STATE_HOMING | STATE_FORCESERVO | STATE_PROBING))) {
      if (sys.state == STATE_QUEUED) {
        sys.state = STATE_CYCLE;
        st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
        st_wake_up();
        if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
          sys.flags |= SYSFLAG_AUTOSTART; // Re-enable auto start after feed hold.
        } else {
          sys.flags &= ~SYSFLAG_AUTOSTART; // Reset auto start per settings.
        }
      }
      bit_false(SYS_EXEC,EXEC_CYCLE_START);
    }

    // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
    // runtime command execution in the main program, ensuring that the planner re-plans safely.
    // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
    // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      if ( plan_get_current_block() ) { sys.state = STATE_QUEUED; }
      else { sys.state = STATE_IDLE; }
      bit_false(SYS_EXEC,EXEC_CYCLE_STOP);
    }

  }

  // Overrides flag byte (sys.override) and execution should be installed here, since they
  // are runtime and require a direct and controlled interface to the main stepper program.

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING | STATE_FORCESERVO | STATE_PROBING)) 
    st_prep_buffer(); 
  
  // Clear IO Reset bit.
  IO_RESET_PORT &= (~IO_RESET_MASK);

}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
  // Check and set auto start to resume cycle after synchronize and caller completes.
  if (sys.state == STATE_CYCLE) { sys.flags |= SYSFLAG_AUTOSTART; }
  while (plan_get_current_block() || (sys.state == STATE_CYCLE)) {
    protocol_execute_runtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  }
}


// Auto-cycle start has two purposes: 1. Resumes a plan_synchronize() call from a function that
// requires the planner buffer to empty (spindle enable, dwell, etc.) 2. As a user setting that
// automatically begins the cycle when a user enters a valid motion command manually. This is
// intended as a beginners feature to help new users to understand g-code. It can be disabled
// as a beginner tool, but (1.) still operates. If disabled, the operation of cycle start is
// manually issuing a cycle start command whenever the user is ready and there is a valid motion
// command in the planner queue.
// NOTE: This function is called from the main loop and mc_line() only and executes when one of
// two conditions exist respectively: There are no more blocks sent (i.e. streaming is finished,
// single commands), or the planner buffer is full and ready to go.
void protocol_auto_cycle_start() {
  if (sys.flags & SYSFLAG_AUTOSTART) { SYS_EXEC |= EXEC_CYCLE_START; }
}

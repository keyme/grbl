/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014 Sungeun K. Jeon, Adam Shelly

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
#include "probe.h"
#include "counters.h"
#include "protocol.h"
#include "stepper.h"
#include "planner.h"
#include "settings.h"
#include "limits.h"
#include "gcode.h"
#include "report.h"
#include "motion_control.h"

#define PROBE_LINE_NUMBER (LINENUMBER_SPECIAL)
struct probe_state probe;

// Idx, Mask, PIN - Note PIN is not the
// pin of the microcontroller, but Port In.
static struct {
  uint8_t idx;
  uint8_t mask;
  volatile uint8_t *ddr;
  // Named in_port rather that pin for clarity
  volatile uint8_t *in_port;
  volatile uint8_t *port;
} sensor_map[] = {
  {
    .idx = MAG_SENSOR,
    .mask = MAGAZINE_ALIGNMENT_MASK,
    .ddr = &MAGAZINE_ALIGNMENT_DDR, 
    .in_port = &MAGAZINE_ALIGNMENT_PIN,
    .port = &MAGAZINE_ALIGNMENT_PORT
  },
  {
    .idx = KEY_SENSOR,
    .mask = (1 << Z_LIMIT_BIT),
    .ddr = &LIMIT_DDR,
    .in_port = &LIMIT_PIN,
    .port = &LIMIT_PORT
  }
};

void set_active_probe(enum e_sensor sensor)
{
  probe.active_sensor = sensor;
}

// Probe pin initialization routine.
void probe_init() 
{
  uint8_t len = sizeof(sensor_map) / sizeof(sensor_map[0]);
  uint8_t idx;
  for (idx = 0; idx < len; idx++) {
    *sensor_map[idx].ddr &= ~(sensor_map[idx].mask);
    *sensor_map[idx].port |= sensor_map[idx].mask;

  }

  probe.active_sensor = E_SENSOR_TYPES;
  probe.probe_reached = 0;
  probe.isprobing = 0;

}

void probe_check()
{
  if (probe_get_active_sensor_state()) {
    // Stop looking for probe
    probe.isprobing = 0;

    // TODO: Report the position where the probe was found
    // using report_probe_parameters(). This should possibly be
    // reported in in probe_loop or probe_move_to_sensor instead of here.
  }

  // Check for ESTOP
  if (ESTOP_PIN & ESTOP_MASK) {
    sys.alarm |= ALARM_ESTOP;
    SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
  }
}

bool probe_loop()
{
  // Start stepper
  st_prep_buffer();
  st_wake_up();

  SYS_EXEC |= EXEC_CYCLE_START;

  // Stay in this loop until alarm or until
  // the probe is found. probe_check() is called
  // from the stepper ISR to check if the active
  // probe is found.
  while (probe.isprobing) {
    // Check for user reset and allow
    // protocol_execute_runtime to run in this loop 
    protocol_execute_runtime();
    
    if(sys.abort)
      return false;

    if (SYS_EXEC & EXEC_RESET) {
      protocol_execute_runtime();
      return false;
    } 

    if (ESTOP_PIN & ESTOP_MASK) {
      sys.alarm |= ALARM_ESTOP;
      SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
      protocol_execute_runtime();
      return false;
    }

    // Check if we never reach probe.
    if ((SYS_EXEC & EXEC_CYCLE_STOP) ) {
      sys.alarm |= ALARM_PROBE_FAIL;
      SYS_EXEC |= EXEC_CRIT_EVENT;
      protocol_execute_runtime();
      return false;
    }
  }

  return true;

}

// This function adds support for gcode G38.2
void probe_move_to_sensor(float * target, float feed_rate, uint8_t invert_feed_rate,
  linenumber_t line_number, enum e_sensor sensor)
{
  // Set the active probe
  probe.active_sensor = sensor;

  if (sys.state != STATE_CYCLE)
    protocol_auto_cycle_start();
  
  // Finish all queued commands
  protocol_buffer_synchronize();

  // Return if system reset has been issued
  if (sys.abort)
    return;

  // Move in a line to the target
  mc_line(target, feed_rate, invert_feed_rate, line_number);

  // TODO: If the probe is already activated, we should look in
  // the oppostie direction that is specified.

  if (sensor == MAG_SENSOR)
    probe.carousel_probe_state = PROBE_ACTIVE;

  // Tell the system we are probing
  probe.isprobing = 1;

  sys.state = STATE_PROBING;
 
  uint8_t probe_fail;
  probe_fail = !probe_loop();
  
  if (sensor == MAG_SENSOR) {
    probe_fail = (probe.carousel_probe_state == PROBE_ACTIVE);
    if (probe_fail)
      memcpy(sys.probe_position, sys.position, sizeof(float) * N_AXIS);
  }

  protocol_execute_runtime();

  if (sys.abort)
    return;
  
  // Prep the new target based on the positon that the probe triggered
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; ++idx) {
    target[idx] = (float)sys.probe_position[idx] / settings.steps_per_mm[idx];
  }

  protocol_execute_runtime();
  
  // Force kill steppers and reset
  // step segment buffer
  st_reset();
  plan_reset();
  plan_sync_position();

  SYS_EXEC |= EXEC_CYCLE_START;

  // Complete pull-off action
  protocol_buffer_synchronize();

  // Did not complete. Alarm state set by mc_alarm
  if (sys.abort)
    return;

  gc_sync_position();

  sys.state = STATE_IDLE;
  st_go_idle();

  request_eol_report();
  report_probe_parameters(probe_fail);
  
}

// This function monitors the carousel magazine alignment probe
// which is used to move to a specified magazine.
void probe_carousel_monitor()
{
  uint8_t probe_on = probe_get_carousel_state();
  if (probe.carousel_probe_state == PROBE_ACTIVE && probe_on) {
    probe.carousel_probe_state = PROBE_OFF;
    memcpy(sys.probe_position, sys.position, sizeof(float) * N_AXIS);
    SYS_EXEC |= EXEC_FEED_HOLD;
  }

  if (ESTOP_PIN & ESTOP_MASK) {
    sys.alarm |= ALARM_ESTOP;
    SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
  }

}


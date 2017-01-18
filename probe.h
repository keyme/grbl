/*
  probe.h - code pertaining to probing methods
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

#ifndef probe_h
#define probe_h 

#define N_SENSORS 1
// Values that define the probing state machine.  
#define PROBE_OFF     0 // No probing. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

enum e_sensor {
  MAG_SENSOR = 0,
  // For now, this is mapped to the gripper's home sensor, it should be changed
  // when we get new hardware to support key measurements with probing
  KEY_SENSOR_FRONT,
  KEY_SENSOR_BACK,
  E_SENSOR_TYPES
};

struct probe_state {
  enum e_sensor active_sensor;  // The currently active probe seonsor
  volatile uint8_t probe_reached;  // Flag to indicate if active probe is reached
  uint8_t isprobing;
  volatile uint8_t carousel_probe_state;
};

extern struct probe_state probe;

// Probe pin initialization routine.
void probe_init();

// Plan a probe move to a probe sensor on an axis in a given direction
void probe_move_to_sensor(float * target, float feed_rate, uint8_t invert_feed_rate,
  linenumber_t line_number, enum e_sensor sensor);

// Used to set active probe to look for
void set_active_probe(enum e_sensor sensor);

// Called from stepper ISR - needs to be very efficient
void probe_check();

// Returns active probe state
#define ACTIVE_SENSOR_MASK (sensor_map[probe.active_sensor].mask)
#define ACTIVE_SENSOR_PIN (*sensor_map[probe.active_sensor].in_port)
#define probe_get_active_sensor_state() (!(ACTIVE_SENSOR_MASK & ACTIVE_SENSOR_PIN))

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
#define probe_get_carousel_state() (!(PROBE_PIN & PROBE_MASK))

// Monitors ACTIVE probe pin state and records the system position
// when detected. Called by the stepper ISR each ISR tick.
void probe_state_monitor();

// Monitors CAROUSEL probe pin state
void probe_carousel_monitor();
#endif

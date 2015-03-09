/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon
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
#include "settings.h"
#include "protocol.h"
#include "planner.h"
#include "stepper.h"
#include "motion_control.h"
#include "limits.h"
#include "report.h"

#define HOMING_AXIS_SEARCH_SCALAR  1.1  // Axis search distance multiplier. Must be > 1.


limit_t limits={0};

static linenumber_t homing_line_number;  //autoincremented every homing cycle, sent w/ high bit set.
                                  //odd numbers are the switch position, 
                                  //even numbers are pulloff complete

// Initializes hardware.
void limits_init() 
{

  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins

  if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) {
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
  } else {
    LIMIT_PORT |= (LIMIT_MASK);  // Enable internal pull-up resistors. Normal high operation.
  }
  //TODO: test this method  inplace of master clock for probe debounce
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
  #endif
  homing_line_number = 1;
  limits_configure(); 
}


//Resets enable state
void limits_configure(){
  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    limits_enable(LIMIT_MASK & HARDSTOP_MASK,0);
  } else {
    limits_disable();
  }
}


void limits_enable(uint8_t axes, uint8_t expected) {
  //    LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
  limits.expected = bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)?~expected:expected;
  limits.active = axes<<LIMIT_BIT_SHIFT;
}


void limits_disable()
{
  //LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
  limits.expected = 0;
  limits.active = 0;
}


// Limit checking moved to stepper ISR. 
// limits_enable() sets limits.active and limits.expected flags.
// stepper stops an axis whenever LIMIT_PIN&limits.active != limits.expected,
//   in this case, it also clears limits.is_homing to signal homing routine to continue.
//   If not homing, it does a hard reset and sets the alarm.state
//   (See `must_stop` section of ISR TIMER1_COMPA_vect)

//  TODO: do we need special handling if already in an alarm state or in-process of executing an alarm.
// Old comments said:
  // Ignore limit switches When in the alarm state:
  // Grbl should have been reset or will force a reset, so any pending 
  // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
  // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
  // limit setting if their limits are constantly triggering after a reset and move their axes.


// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock 
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically 
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort runtime command can interrupt this process.
void limits_go_home(uint8_t cycle_mask) 
{
  if (sys.abort || !cycle_mask) { return; } // Block if system reset has been issued.

  float homing_rate;
  // Initialize homing in search mode to quickly engage the specified cycle_mask limit switches.
  uint8_t approach = ~0;  //approach has all bits set (negative dir) or none (positive)
  uint8_t idx;
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t n_active_axis = 0;
  float target[N_AXIS];

  uint8_t flipped = settings.homing_dir_mask>>X_DIRECTION_BIT;  //assumes keyme configuration.
  //replace with an instance of `if (bitistrue(h_d_m,X_DIRECTION_BIT)) { flipped|=1<<X_AXIS;}` 
  //for each axis if the bits line up differently

  // Determine travel distance to the furthest homing switch based on user max travel settings.
  float min_seek_rate = 1e9; //arbitrary maximum=1km/s, will be reduced by axis setting below
  float max_travel = 0;

  for (idx=0; idx<N_AXIS; idx++){
    if (bit_istrue(cycle_mask,bit(idx))) {
      n_active_axis++;
      max_travel = max(max_travel,settings.max_travel[idx]);
      min_seek_rate = min(min_seek_rate,settings.homing_seek_rate[idx]);
    }
  }
  max_travel *= HOMING_AXIS_SEARCH_SCALAR; // Ensure homing switches engaged by over-estimating max travel.
  max_travel += settings.homing_pulloff;
  homing_rate = min_seek_rate;
  // Adjust total rate so individual axes all move at desired homing rate.
  homing_rate *= sqrt(n_active_axis);

  plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.

  do {
    // Set target location and rate for active axes.
    // and reset homing axis locks based on cycle mask.  
    uint8_t axislock = 0;
    float travel = (approach)?max_travel:settings.homing_pulloff;
    
    for (idx=0; idx<N_AXIS; idx++) {
      if (bit_istrue(cycle_mask,bit(idx))) {
        axislock |= (1<<(X_STEP_BIT+idx)); //assumes axes are in bit order.
        if ((flipped&(1<<idx))^approach) { target[idx] = -travel; }
        else {                             target[idx] = travel; }
      } 
      else {                               target[idx] = 0; 
      } 
    }

    // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
    plan_buffer_line(target, homing_rate, false, LINENUMBER_EMPTY_BLOCK); // Bypass mc_line(). Directly plan homing motion.

	 // axislock bit is high if axis is homing, so we only enable checking on moving axes.
    limits_enable(axislock,~approach);  //expect 0 on approach (stop when 1). vice versa for pulloff
    limits.ishoming =1;

    st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
    st_wake_up(); // Initiate motion

    do {
      st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.
      // Check only for user reset. No time to run protocol_execute_runtime() in this loop.
      protocol_execute_runtime();
      if (SYS_EXEC & EXEC_RESET) { protocol_execute_runtime(); return; }

      // Check if we never reached limit switch.  call it a Probe fail.
      if (SYS_EXEC & EXEC_CYCLE_STOP) {
        SYS_EXEC|=EXEC_CRIT_EVENT;
        protocol_execute_runtime();
        return;
      }
    } while (limits.ishoming);  //stepper isr sets this when limit is hit

    limits_disable();
    st_reset(); // Immediately force kill steppers and reset step segment buffer.
    plan_reset(); // Reset planner buffer. Zero planner positions. Ensure homing motion is cleared.

    delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.

    // Reverse direction and reset homing rate for locate cycle(s).
    homing_rate = settings.homing_feed_rate;
    approach = ~approach; //toggle all bits

  } while (n_cycle-- > 0);

  //force report of known position for compare to zero.
  linenumber_insert(LINENUMBER_SPECIAL|homing_line_number++);
  request_eol_report();
  protocol_execute_runtime();

  // The active cycle axes should now be homed and machine limits have been located. By 
  // default, grbl defines machine space as all negative, as do most CNCs. Since limit switches
  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
  // set up pull-off maneuver from axes limit switches that have been homed. This provides
  // some initial clearance off the switches and should also help prevent them from falsely
  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
  for (idx=0; idx<N_AXIS; idx++) {
    // Set up pull off targets and machine positions for limit switches homed in the negative
    // direction, rather than the traditional positive. Leave non-homed positions as zero and
    // do not move them.
    if (cycle_mask & bit(idx)) {
      if ( settings.homing_dir_mask & get_direction_mask(idx) ) {
        target[idx] = settings.max_travel[idx];
        sys.position[idx] = lround((settings.homing_pulloff+settings.max_travel[idx])*settings.steps_per_mm[idx]);
      } else {
        sys.position[idx] = -settings.homing_pulloff*settings.steps_per_mm[idx];
        target[idx] = 0;
      }
      if (settings.homing_pulloff == 0.0) {request_eol_report(); } //force report if we are not going to move  TODO TEST
    } else { // Non-active cycle axis. Set target to not move during pull-off.
      target[idx] = (float)sys.position[idx]/settings.steps_per_mm[idx];
    }
  }
  plan_sync_position(); // Sync planner position to current machine position for pull-off move.

  plan_buffer_line(target, min_seek_rate, false, LINENUMBER_SPECIAL|homing_line_number++); // Bypass mc_line(). Directly plan motion.

  // Initiate pull-off using main motion control routines.
  // TODO : Clean up state routines so that this motion still shows homing state.
  sys.state = STATE_QUEUED;
  SYS_EXEC |= EXEC_CYCLE_START;
  protocol_execute_runtime();
  protocol_buffer_synchronize(); // Complete pull-off motion.

  // Set system state to homing before returning.
  sys.state = STATE_HOMING;
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all positive space, and the system is in normal operation.
void limits_soft_check(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    if ((target[idx] < 0 || target[idx] > settings.max_travel[idx])  &&  
        (get_step_mask(idx)&HARDSTOP_MASK)) {   //if rotary axis, don't check

      // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within 
      // workspace volume so just come to a controlled stop so position is not lost. When complete
      // enter alarm mode.
      if (sys.state == STATE_CYCLE) {
        SYS_EXEC |= EXEC_FEED_HOLD;
        do {
          protocol_execute_runtime();
          if (sys.abort) { return; }
        } while ( sys.state != STATE_IDLE || sys.state != STATE_QUEUED);
      }

      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      SYS_EXEC |= (EXEC_ALARM | EXEC_CRIT_EVENT); // Indicate soft limit critical event
      protocol_execute_runtime(); // Execute to enter critical event loop and system abort
      return;
    }
  }
}

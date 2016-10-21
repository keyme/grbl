/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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
#include "nuts_bolts.h"
#include "stepper.h"
#include "settings.h"
#include "planner.h"
#include "probe.h"
#include "limits.h"
#include "motion_control.h"
#include "report.h"
#include "magazine.h"

#include "spi.h"

// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

//SPI drivers
#define SPI_ADDRESS_MASK        0x7
#define SPI_RW_BIT              7
// For use with the SPI driver chip
typedef enum {
  XTABLE = 0,
  YTABLE,
  GRIPPER,
  CAROUSEL
} steppers_t;

static const uint8_t scs_pin_lookup[4] = {
  SCS_XTABLE_PIN,
  SCS_YTABLE_PIN,
  SCS_GRIPPER_PIN,
  SCS_CAROUSEL_PIN
  
};

// Initialization values for spi stepper drivers
// Format: MSB, LSB
static const uint8_t stepper_init_registers[4][18] = {
  {
    //XTABLE
    0x0C, 0x11, // CTRL,   DTIME=11, ISGAIN=00, EXSTALL=0, MODE=0010, RSTEP=0, RDIR=0, ENBL=1
    0x11, 0xFF, // TORQUE, SMPLTH=001, TORQUE=0xFF
    0x20, 0x30, // OFF,    PWMMODE=0, TOFF=0x30
    0x30, 0x80, // BLANK,  ABT=0, TBLANK=0x80
    0x41, 0x10, // DECAY,  DECMOD=001, TDECAY=0x10
    0x50, 0x40, // STALL,  VDIV=00, SDCNT=00, SDTHR=0x40
    0x6A, 0x59, // DRIVE,  IDRIVEP=10, IDRIVEN=10, TDRIVEP=01, TDRIVEN=01, OCPDEG=10, OCPTH=01
    0x70, 0x00  // STATUS, init all status flags to 0 
  },
  {
    //YTABLE
    0x0C, 0x11, // CTRL,   DTIME=11, ISGAIN=00, EXSTALL=0, MODE=0010, RSTEP=0, RDIR=0, ENBL=1	
    0x11, 0xFF, // TORQUE, SMPLTH=001, TORQUE=0xFF
    0x20, 0x30, // OFF,    PWMMODE=0, TOFF=0x30
    0x30, 0x80, // BLANK,  ABT=0, TBLANK=0x80
    0x41, 0x10, // DECAY,  DECMOD=001, TDECAY=0x10
    0x50, 0x40, // STALL,  VDIV=00, SDCNT=00, SDTHR=0x40
    0x6A, 0x59, // DRIVE,  IDRIVEP=10, IDRIVEN=10, TDRIVEP=01, TDRIVEN=01, OCPDEG=10, OCPTH=01
    0x70, 0x00  // STATUS, init all status flags to 0 
  },
  {
    //GRIPPER
    0x0C, 0x11, //  CTRL,   DTIME=11, ISGAIN=00, EXSTALL=0, MODE=0010, RSTEP=0, RDIR=0, ENBL=1  
    0x11, 0xFF, //  TORQUE, SMPLTH=001, TORQUE=0xFF
    0x20, 0x30, //  OFF,    PWMMODE=0, TOFF=0x30
    0x30, 0x80, //  BLANK,  ABT=0, TBLANK=0x80
    0x41, 0x10, //  DECAY,  DECMOD=001, TDECAY=0x10
    0x50, 0x40, //  STALL,  VDIV=00, SDCNT=00, SDTHR=0x40
    0x6A, 0x59, //  DRIVE,  IDRIVEP=10, IDRIVEN=10, TDRIVEP=01, TDRIVEN=01, OCPDEG=10, OCPTH=01   
    0x70, 0x00  //  STATUS, init all status flags to 0 

  },
  {
    //CAROUSEL
    0x0C, 0x11, //  CTRL,   DTIME=11, ISGAIN=00, EXSTALL=0, MODE=0010, RSTEP=0, RDIR=0, ENBL=1  
    0x11, 0xFF, //  TORQUE, SMPLTH=001, TORQUE=0xFF
    0x20, 0x30, //  OFF,    PWMMODE=0, TOFF=0x30
    0x30, 0x80, //  BLANK,  ABT=0, TBLANK=0x80
    0x41, 0x10, //  DECAY,  DECMOD=001, TDECAY=0x10
    0x50, 0x40, //  STALL,  VDIV=00, SDCNT=00, SDTHR=0x40
    0x6A, 0x59, //  DRIVE,  IDRIVEP=10, IDRIVEN=10, TDRIVEP=01, TDRIVEN=01, OCPDEG=10, OCPTH=01   
    0x70, 0x00  //  STATUS, init all status flags to 0 
  }
};

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)



// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct {
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
  uint8_t do_status;         //true for last segment of a block - used to force reporting
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y,
           counter_z,
           counter_c;

  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t step_outbits;         // The next stepping-bits to be output
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #else
    uint32_t *steps;
  #endif

  uint16_t step_count;       // Steps remaining in line segment motion
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.

  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float steps_remaining;
  float step_per_mm;           // Current planner block step/millimeter conversion scalar
  float req_mm_increment;
  float dt_remainder;

  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)
} st_prep_t;
static st_prep_t prep;

static uint64_t st_shutdown_start;
static uint16_t st_shutdown_delay;  //ms (max = 32767)

/*    BLOCK VELOCITY PROFILE DEFINITION
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity

  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time --  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters
  are shown and defined in the above illustration.
*/


//disable stepper output (0 to enable)
void st_disable(uint8_t disable, uint8_t mask) {
  if (mask & STEPPERS_LONG_LOCK_MASK) st_shutdown_start = 0;  //clear pending shutdown if we are enabling, or if it has pent.
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { disable = !disable; } // Apply pin invert.
  if (disable) { STEPPERS_DISABLE_PORT |= (STEPPERS_DISABLE_MASK&mask); }
  else { STEPPERS_DISABLE_PORT &= ~(STEPPERS_DISABLE_MASK&mask); }
}

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up()
{
  // Enable all stepper drivers.
  st_disable(false,~0);

  if (sys.state & (STATE_CYCLE | STATE_HOMING)){
    // Initialize stepper output bits
    st.dir_outbits = settings.dir_invert_mask;
    st.step_outbits = settings.step_invert_mask;

    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    #endif

    // Enable Stepper Driver Interrupt
    TIMSK4 |= (1<<OCIE4A);
  }
}


// Stepper shutdown
void st_go_idle()
{
  // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  TIMSK4 &= ~(1<<OCIE4A); // Disable Timer4 interrupt
  TCCR4B = (TCCR4B & ~((1<<CS42) | (1<<CS41))) | (1<<CS40); // Reset clock to no prescaling.
  busy = false;

  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
  bool do_disable = false; // Keep enabled.
  uint8_t mask = ~0; //all axes
  if ((sys.state != STATE_HOMING)){// && (sys.state != STATE_FORCESERVO)) {
    if (bit_istrue(SYS_EXEC, EXEC_ALARM)) {
      do_disable = true;  //disable all on alarm.
    }
    else if (settings.stepper_idle_lock_time != 0xff) { //else not always on
      st_shutdown_delay = settings.stepper_idle_lock_time*STEPPERS_LOCK_TIME_MULTIPLE;
      st_shutdown_start = masterclock|1; //use nearest odd number to handle rare case of mc==0

      delay_ms(settings.stepper_idle_lock_time);
      do_disable = true;  //disable most axes now.
      mask = ~STEPPERS_LONG_LOCK_MASK;
    }
  }
  st_disable(do_disable, mask);

}

void st_check_disable() {
  if (st_shutdown_start && ((uint16_t)(masterclock - st_shutdown_start) > st_shutdown_delay)) {
    st_disable(true, STEPPERS_LONG_LOCK_MASK);  //disable long-dwell axes after timeout
  }
}

//Called from ISR(TIMER4_COMPA_vect) - needs to be very efficient 
void st_limit_check(){
  // While homing or if hard limits enabled

  // Limit checking performed here

  // LIMIT_PIN - Input buffer of port to which home sensors are connected
  // limits.expected - approach set in limits.c. limits.expected stores what the pins in LIMIT_PIN
  // should be when the axes are moving and have not reached the home sensor
  // limits.active - active bit is set in limits.active if axis is homing
  // If home sensor is reached in LIMIT_PIN and the pin is different than
  // that of limits.expected and the axis is homing as set in limits.active,
  // then set the bit for that pin in must_stop, to indicate that the corresponding
  // axis should stop

  //Disable magazine gap checking if carousel has finished homing, but other axes are still homing
  if( !(limits.ishoming & (1 << C_AXIS)) && limits.ishoming){
    limits.mag_gap_check = 0 ;
  }
  
  uint8_t must_stop = (( (LIMIT_PIN) ^ limits.expected) & limits.active);
  if (must_stop) {
    st.step_outbits &= ~(must_stop >> LIMIT_BIT_SHIFT);
    limits.ishoming &= ~(must_stop >> LIMIT_BIT_SHIFT); // If an axis is done homing, clear the corresponding bit in limits.ishoming     
 
  if (!limits.ishoming) {
      request_report(REQUEST_STATUS_REPORT|REQUEST_LIMIT_REPORT,LINENUMBER_EMPTY_BLOCK);
  } else { 
    bit_true(sys.state, STATE_HOME_ADJUST);

  }

    //if limits made but not homing , servoing, or alarmed already: critical alarm.
    if ( !(sys.state & (STATE_ALARM|STATE_HOMING)) && !(sys.state & (STATE_ALARM|STATE_FORCESERVO)) &&
         bit_isfalse(SYS_EXEC,EXEC_ALARM)) {
      mc_reset(); // Initiate system kill.
      // Indicate hard limit critical event, print limits
      sys.alarm |= ALARM_HARD_LIMIT;
      request_report(REQUEST_LIMIT_REPORT, (EXEC_ALARM | EXEC_CRIT_EVENT));
    }
  }

 
}

/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without affecting the algorithm's
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing
   level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing
   issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed.
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease
   CPU overhead with bitshift integer operations.
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.

   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick,
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in
   ISR is 5usec typical and 25usec maximum, well below requirement.
   NOTE: This ISR expects at least one step to be executed per segment.
*/
// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated
// with probing and homing cycles that require true real-time positions.
ISR(TIMER4_COMPA_vect)
{
  TIME_OFF(time_STEP_ISR); // Debug: Used to time ISR
  if (busy) {  // The busy-flag is used to avoid reentering this interrupt
    TIME_ON(time_STEP_ISR);
    return;
  }

  // Set the direction pins a couple of nanoseconds before we step the steppers
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

  // Then pulse the stepping pins
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
  #else  // Normal operation
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
  #endif

  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer4 prescaler.
  TCNT0 = st.step_pulse_time; // Reload Timer0 counter
  TCCR0B = (1<<CS01); // Begin Timer0. Full speed, 1/8 prescaler

  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time.
         // NOTE: The remaining code in this ISR will finish before returning to main program.

  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        TCCR4B = (TCCR4B & ~(0x07<<CS40)) | (st.exec_segment->prescaler<<CS40);
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      OCR4A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // Initialize Bresenham line and distance counters
        st.counter_x = (st.exec_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
                  st.counter_c = st.counter_x;
      }

      st.dir_outbits = st.exec_block->direction_bits ^ settings.dir_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
        st.steps[C_AXIS] = st.exec_block->steps[C_AXIS] >> st.exec_segment->amass_level;
                #else
                  st.steps = st.exec_block->steps;
      #endif


    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true(SYS_EXEC,EXEC_CYCLE_STOP); // Flag main program for cycle end
      TIME_ON(time_STEP_ISR);
      return; // Nothing to do but exit.
    }
  }
  // Check probing state.
  probe_state_monitor();

  if(settings.mag_gap_enabled) {
   // Monitor magazine probe to look for missing magazines on carousel
   magazine_gap_monitor();
  }

  // Reset step out bits.
  st.step_outbits = 0;

  // Execute step displacement profile by Bresenham line algorithm
  st.counter_x += st.steps[X_AXIS];

  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
    else { sys.position[X_AXIS]++; }
  }

  st.counter_y += st.steps[Y_AXIS];

  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
    else { sys.position[Y_AXIS]++; }
  }

  st.counter_z += st.steps[Z_AXIS];

  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
    else { sys.position[Z_AXIS]++; }
  }

  st.counter_c += st.steps[C_AXIS];

  if (st.counter_c > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<C_STEP_BIT);
    st.counter_c -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<C_DIRECTION_BIT)) { sys.position[C_AXIS]--; }
    else { sys.position[C_AXIS]++; }
  }

  st_limit_check(); //Check for limits

  // This checks if desired force value is met while bumping key.
  // Once value is reached, gripper motor will stop.
  // Threshold Value can be tweaked for desired target force value
  int16_t error = (int16_t)analog_voltage_readings[FORCE_VALUE_INDEX] - (int16_t)force_target_val;
  if (limits.isservoing){
    if (abs(error)<=GRIPPER_FORCE_THRESHOLD){ //check if force servoing is active
      limits.isservoing = 0;
      request_report(REQUEST_STATUS_REPORT|REQUEST_LIMIT_REPORT,LINENUMBER_EMPTY_BLOCK);
    }
    // The next two conditionals check if the gripper motor went past the force threshold.
    // This could happen if the motor moves too quickly or if the voltage is not checked
    // often enough.
    else if ((travel_servo>0) && (error>GRIPPER_FORCE_THRESHOLD)){
      // Situation: Closing the gripper, but we skip past the threshold.
      limits.isservoing = 0;
      request_report(REQUEST_STATUS_REPORT|REQUEST_LIMIT_REPORT,LINENUMBER_EMPTY_BLOCK);
    }
    else if ((travel_servo<0) && (error<-GRIPPER_FORCE_THRESHOLD)){
      // Situation: Opening the gripper, but we skip past the threshold.
      limits.isservoing = 0;
      request_report(REQUEST_STATUS_REPORT|REQUEST_LIMIT_REPORT,LINENUMBER_EMPTY_BLOCK);
    }
  }

  st.step_count--; // Decrement step events count
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    if (st.exec_segment->do_status) { request_eol_report();  }

    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= settings.step_invert_mask;  // Apply step port invert mask
  busy = false;
  TIME_ON(time_STEP_ISR);
  return;
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer4 COMPA interrupt and independently
   finish, if Timer4 is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/
// This interrupt is enabled by ISR_TIMER4_COMPAREA when it sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
ISR(TIMER0_OVF_vect)
{
  // Reset stepping pins (leave the direction pins)
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (settings.step_invert_mask & STEP_MASK);
  TCCR0B = 0; // Disable Timer0 to prevent re-entering this interrupt when it's not needed.
}
#ifdef STEP_PULSE_DELAY
  // This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
  // initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
  // will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
  // The new timing between direction, step pulse, and step complete events are setup in the
  // st_wake_up() routine.
  ISR(TIMER0_COMPA_vect)
  {
    STEP_PORT = st.step_bits; // Begin step pulse.
  }
#endif


// Reset and clear stepper subsystem variables
void st_reset()
{
  // Initialize stepper driver idle state.
  st_go_idle();

  memset(&prep, 0, sizeof(prep));
  memset(&st, 0, sizeof(st));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer

  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

}

void spi_read_driver_register(uint8_t addr, uint8_t * dataout , steppers_t stepper)
{
  //For read, MSB should be 1
  uint8_t tx_data[2];
  tx_data[0] = (1 << SPI_RW_BIT);
  tx_data[0] |= (addr & SPI_ADDRESS_MASK) << 4;

  bit_true(SCS_PORT, 1 << scs_pin_lookup[(uint8_t)stepper]); // Chip select high
  spi_transact_array(tx_data, dataout, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[(uint8_t)stepper]); // Chip select low
 
}

void spi_driver_setup(steppers_t stepper)
{
  uint8_t idx = 0;
  for(idx = 0; idx <= 14; idx += 2) {
    uint8_t tx_buf[2];
    tx_buf[0] = stepper_init_registers[(uint8_t)stepper][idx];
    tx_buf[1] = stepper_init_registers[(uint8_t)stepper][idx + 1];

    bit_true(SCS_PORT, 1 << scs_pin_lookup[(uint8_t)stepper]); // Chip select high
    spi_write(tx_buf, 2);
    bit_false(SCS_PORT, 1 << scs_pin_lookup[(uint8_t)stepper]); // Chip select low

  }
}


void keyme_init() 
{
  // PORTG0 for drive enable
  ESTOP_DDR  |= (1<<RUN_ENABLE_BIT);    //set enable as outupt
  ESTOP_DDR  &= ~(ESTOP_BIT);           //estop as input
  ESTOP_PORT |= (1<<RUN_ENABLE_BIT);  //allow motors to run
  ESTOP_PORT &= ~(ESTOP_BIT);           //estop input normal-low

  // Microstepping

  #ifdef SPI_STEPPER_DRIVER
    //Initialise SPI Stepper drivers
    spi_driver_setup(XTABLE);
    spi_driver_setup(YTABLE);
    spi_driver_setup(GRIPPER);
    spi_driver_setup(CAROUSEL);
  #else
    MS_DDR = MS_MASK; //all output
    MS_PORT = settings.microsteps & MS_MASK;
  #endif 

  // Phase Current Decay
  PFD_DDR = PFD_MASK; //all output
  uint8_t axis, value=0;
  for (axis=0; axis<N_AXIS; axis++) {
    value |= SET_DECAY_MODE(axis, settings.decay_mode);
  }
  PFD_PORT = value & PFD_MASK;

  // MVOLT is used for voltage sensing
  // TODO:  read on command.  (create command)
  MVOLT_DDR &= ~(MVOLT_MASK);

  // Setup CCTRL (current control) ports
  // TODO: make this output PWMs with configurable values
  // for now, just turning on to full power
  CCTRL_DDR |= (1<<CCTRL_CG_BIT);
  CCTRL_DDR |= (1<<CCTRL_XY_BIT);

  // TODO: Replace this with analog out code
  CCTRL_PORT |= (1<<CCTRL_CG_BIT);
  CCTRL_PORT |= (1<<CCTRL_XY_BIT);

  // Setup IO Reset Port
  IO_RESET_DDR |= IO_RESET_MASK;
  IO_RESET_PORT &= ~IO_RESET_MASK; //don't reset


  // Setup hardware PWM timing on Timer3 to control OC3 A,B and C
  // with pwm signals

  // PWM Setup, fast pwm mode, 8 bit:
  //  WGM33=0, on register B
  //  WGM32=1, on register B
  //  WGM31=0, on register A
  //  WGM30=1, on register A

  // No prescaling to get fastest speed
  //  CS32=0, register B
  //  CS31=0, register B
  //  CS30=1, register B

  // Enable non inverting fast PWM compare
  //  Enable all three comparators (A, B, C)
  //  COM3A1=1, on register A
  //  COM3A0=0, on register A
  //  COM3B1=1, on register A
  //  COM3B0=0, on register A
  //  COM3C1=1, on register A
  //  COM3C0=0, on register A

  TCCR3A = (1<<COM3A1)+(0<<COM3A0) + (1<<COM3B1)+(0<<COM3B0) + (1<<COM3C1)+(0<<COM3C0) + (0<<WGM31)+(1<<WGM30);
  TCCR3B = (0<<WGM33)+(1<<WGM32) + (0<<CS32)+(0<<CS31)+(1<<CS30);
  TCCR3C = 0;

  // Set all three pins as OUTPUT
  PWM_OUT_DDR |= 1<<PWM_OUT_FSENSE_BIT;
  PWM_OUT_DDR |= 1<<PWM_OUT_XY_CTRL_BIT;
  PWM_OUT_DDR |= 1<<PWM_OUT_CG_CTRL_BIT;

  // Set OLD PWM Sensor Drive as INPUT so it's not driving anything
  // TODO: remove this as un-needed on Rev5 boards
  FSCTRL_DDR |= 0<<FSCTRL_BIT;

  // With the comparators and timers set up, use OCR3xL to set compare value
  // In 8 bit mode, only the Low register is needed
  OCR3AL = 254; // 5V, full current
  OCR3BL = settings.force_sensor_level; // 1.5V default
  OCR3CL = 254; // 5V, full current

  /*
  // THIS WAS TEST CODE TO TRY TO DRIVE A SOFTWARE PWM
  // Saved for posterity and maybe future use

  // Setup PWM out on timer3 for force sensor sensitivity
  // Control register A
  // We want normal port operation since the desired
  // output pin is not connected to timer4 direction
  // Fast PWM mode: WGM40 = 1
  // Normal port operation: COM4Ax = 00
  PWM_OUT_TCCRA = (1<<WGM21)|(0<<COM4A0)|(0<<COM4A1);

  // Control register B
  // Fast PWM mode: WGM42 = 1
  // Set PWM timer clock
  // CS4x, 3 bits
  // CS42: 0, CS41: 0, CS40: 1 = sets clock, no prescaling
  // CS42: 0, CS41: 1, CS40: 1 = sets clock, 64 prescaling
  PWM_OUT_TCCRB = (0<<WGM42)|(1<<CS42)|(0<<CS41)|(0<<CS40);

  // Configure the right bit as output for PWM output,
  // initialize to off
  PWM_OUT_DDR |= (0<<PWM_OUT_BIT);
  PWM_OUT_PORT |= (0<<PWM_OUT_BIT);

  // Maybe this? Not sure if it works in fast PWM mode
  //TCCR4C = (1<<FOC4A);

  // In FPWM mode compare is an 8 bit comparison value
  // We want 1V output
  // 0x33 is 31 of 255
  OCR4A = 200;
  OCR4B = 50;

  // Enable interrupt for A comparison
  //TIMSK4 = (1<<OCIE4A)|(1<<TOIE4);
  */
}

/* This is used to change the Force Sensor Value in the
   specific PWM register.*/
void adjustForceSensorPWM(){
  OCR3BL = settings.force_sensor_level;
}

/*
// ISR's for above test code to drive a software PWM
// Saved for posterity and maybe future use

//volatile int pwm_out_state = 0;

// ISR to handle PWM for output
ISR(TIMER4_COMPA_vect)
{
  PWM_OUT_PORT = (0<<PWM_OUT_BIT);
}

ISR(TIMER4_OVF_vect)
{
  PWM_OUT_PORT = (1<<PWM_OUT_BIT);
}
*/

// Initialize and start the stepper motor subsystem
void stepper_init()
{
  // Configure step and direction interface pins
  STEP_DDR |= STEP_MASK;
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | settings.step_invert_mask;
  STEPPERS_DISABLE_DDR |= STEPPERS_DISABLE_MASK;
  DIRECTION_DDR |= DIRECTION_MASK;
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | settings.dir_invert_mask;

  // The following used to be Timer 1 but was moved to Timer 4 in order
  // for the ADC to use Timer 1
  // Configure Timer 4: Stepper Driver Interrupt
  TCCR4B &= ~(1<<WGM43); // waveform generation = 0100 = CTC
  TCCR4B |=  (1<<WGM42);
  TCCR4A &= ~((1<<WGM41) | (1<<WGM40));
  TCCR4A &= ~((1<<COM4A1) | (1<<COM4A0) | (1<<COM4B1) | (1<<COM4B0)); // Disconnect OC4 output

  // Configure Timer 0: Stepper Port Reset Interrupt
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR0A = 0; // Normal operation
  TCCR0B = 0; // Disable Timer0 until needed
  TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt
  #endif
  //Setup KeyMe specific ports
  keyme_init();

}


// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.flag_partial_block = true;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load new velocity profile.
  }
}


/* Prepares step segment buffer. Continuously called from main program.

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it.
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer()
{
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

    // Determine if we need to load a new planner block or if the block has been replanned.
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block
      if (pl_block == NULL) { return; } // No planner blocks. Exit.

      // Check if the segment buffer completed the last planner block. If so, load the Bresenham
      // data for the block. If not, we are still mid-block and the velocity profile was updated.
      if (prep.flag_partial_block) {
        prep.flag_partial_block = false; // Reset flag
      } else {
        // Increment stepper common data index to store new planner block data.
        if ( ++prep.st_block_index == (SEGMENT_BUFFER_SIZE-1) ) { prep.st_block_index = 0; }

        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;

        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
          st_prep_block->steps[C_AXIS] = pl_block->steps[C_AXIS];
          st_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[C_AXIS] = pl_block->steps[C_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;

        prep.dt_remainder = 0.0; // Reset for new planner block

        if (sys.state == STATE_HOLD) {
          // Override planner block entry speed and enforce deceleration during feed hold.
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }

      /* ---------------------------------------------------------------------------------
         Compute the velocity profile of a new planner block based on its entry and exit
         speeds, or recompute the profile of a partially-completed planner block if the
         planner has updated it. For a commanded forced-deceleration, such as from a feed
         hold, override the planner velocities and decelerate to the target exit speed.
      */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state == STATE_HOLD) { // [Forced Deceleration to Zero Velocity]
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
        // the planner block profile, enforcing a deceleration to zero speed.
        prep.ramp_type = RAMP_DECEL;
        // Compute decelerate distance relative to end of block.
        float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0.0) {
          // Deceleration through entire planner block. End of feed hold is not in this block.
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist; // End of feed hold.
          prep.exit_speed = 0.0;
        }
      } else { // [Normal Operation]
        // Compute or recompute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = pl_block->millimeters;
        prep.exit_speed = plan_get_exec_block_exit_speed();
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
            prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type
              prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
              if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) {
                // Cruise-deceleration or cruise-only type.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types
                prep.accelerate_until -= inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr);
              }
            } else { // Triangle type
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }
          } else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }

    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];


    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create
      a full segment based on the current ramp conditions. If the segment time is incomplete
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if
      an incomplete segment terminates at the end of the velocity profile, the segment is
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at
      the end of planner block (typical) or mid-block at the end of a forced deceleration,
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0; // Initialize segment time
    float time_var = dt_max; // Time worker variable
    float mm_var; // mm-Distance worker variable
    float speed_var; // Speed worker variable
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL:
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise.
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Deceleration only.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          } // End of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.


    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions.
       However, since floats have only 7.2 significant digits, long moves with extremely
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    float steps_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
    float n_steps_remaining = ceil(steps_remaining); // Round-up current steps remaining
    float last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.

    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.state == STATE_HOLD) {

        // Less than one step to decelerate to zero speed, but already very close. AMASS
        // requires full steps to execute. So, just bail.
        prep.current_speed = 0.0;
        prep.dt_remainder = 0.0;
        prep.steps_remaining = n_steps_remaining;
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps.
        plan_cycle_reinitialize();
        sys.state = STATE_QUEUED;
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(last_n_steps_remaining - steps_remaining); // Compute adjusted step rate inverse
    prep.dt_remainder = (n_steps_remaining - steps_remaining)*inv_rate; // Update segment partial step time

    // Compute CPU cycles per step for the prepped segment.
    uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else
      // Compute step timing and timer prescalar for normal step generation.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.)
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Setup initial conditions for next segment.
    if (mm_remaining > prep.mm_complete) {
      // Normal operation. Block incomplete. Distance remaining in block to be executed.
      pl_block->millimeters = mm_remaining;
      prep.steps_remaining = steps_remaining;
      prep_segment->do_status = 0;
    } else {
      // End of planner block or forced-termination. No more distance to be executed.
      //mark which line this segment belongs to
      prep_segment->do_status = REQUEST_STATUS_REPORT;
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail.
        // NOTE: Currently only feed holds qualify for this scenario. May change with overrides.
        prep.current_speed = 0.0;
        prep.dt_remainder = 0.0;
        prep.steps_remaining = ceil(steps_remaining);
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps.
        plan_cycle_reinitialize();
        sys.state = STATE_QUEUED; // End cycle.

        return; // Bail!
// TODO: Try to move QUEUED setting into cycle re-initialize.

      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        pl_block = NULL;
        plan_discard_current_block();
      }
    }

  }
}



/*
   TODO: With feedrate overrides, increases to the override value will not significantly
     change the current planner and stepper operation. When the value increases, we simply
     need to recompute the block plan with new nominal speeds and maximum junction velocities.
     However with a decreasing feedrate override, this gets a little tricky. The current block
     plan is optimal, so if we try to reduce the feed rates, it may be impossible to create
     a feasible plan at its current operating speed and decelerate down to zero at the end of
     the buffer. We first have to enforce a deceleration to meet and intersect with the reduced
     feedrate override plan. For example, if the current block is cruising at a nominal rate
     and the feedrate override is reduced, the new nominal rate will now be lower. The velocity
     profile must first decelerate to the new nominal rate and then follow on the new plan.
        Another issue is whether or not a feedrate override reduction causes a deceleration
     that acts over several planner blocks. For example, say that the plan is already heavily
     decelerating throughout it, reducing the feedrate override will not do much to it. So,
     how do we determine when to resume the new plan? One solution is to tie into the feed hold
     handling code to enforce a deceleration, but check when the current speed is less than or
     equal to the block maximum speed and is in an acceleration or cruising ramp. At this
     point, we know that we can recompute the block velocity profile to meet and continue onto
     the new block plan.
       One "easy" way to do this is to have the step segment buffer enforce a deceleration and
     continually re-plan the planner buffer until the plan becomes feasible. This can work
     and may be easy to implement, but it expends a lot of CPU cycles and may block out the
     rest of the functions from operating at peak efficiency. Still the question is how do
     we know when the plan is feasible in the context of what's already in the code and not
     require too much more code?
*/

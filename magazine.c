/*
  Not part of Grbl. Written by KeyMe.
*/

#include "system.h"
#include "settings.h"
#include "magazine.h"
#include "limits.h"
#include "protocol.h"
#include "stepper.h"

void magazine_init()
{
  MAGAZINE_ALIGNMENT_DDR &= ~(MAGAZINE_ALIGNMENT_MASK); // Configure as input pin
  MAGAZINE_ALIGNMENT_PORT |= MAGAZINE_ALIGNMENT_MASK; // Enable internal pull-up resistors. Normal high operation
  memcpy(sys.probe_position, sys.position, sizeof(int32_t) * N_AXIS); // Set the magazine alignment position to the current position
}

//KeyMe function
// Monitors the gap in units between mags and throws an alarm if the gap is larger than a
// specified threshold.
void magazine_gap_monitor()
{
  const uint8_t magazine_alignment_on = magazine_get_state();
  // When the probe is detected, copy the current position of all axes
  // into sys.probe_position
  if(magazine_alignment_on){
    memcpy(sys.probe_position, sys.position, sizeof(int32_t) * N_AXIS);
  }

  if(limits.mag_gap_check == 0)
    return;

  // Activate alarm if the gap between the current position and the previous
  // probe position becomes too large. Only do this when the system has already homed
  const int32_t cur_pos = sys.position[C_AXIS];
  const int32_t probe_pos = sys.probe_position[C_AXIS];
  const int32_t delta_pos_mm = abs(cur_pos - probe_pos) / settings.steps_per_mm[C_AXIS];
  if(delta_pos_mm > settings.mag_gap_limit){
    sys.state = STATE_ALARM;
    sys.alarm |= ALARM_MAG_MISSING;
    SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
    st_go_idle();
    protocol_execute_runtime();
    
  }
  return;

}




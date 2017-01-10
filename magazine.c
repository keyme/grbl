/*
  Not part of Grbl. Written by KeyMe.
*/

#include "system.h"
#include "settings.h"
#include "magazine.h"
#include "limits.h"
#include "protocol.h"
#include "stepper.h"
#include "report.h"
#include "gqueue.h"
#include "probe.h"

enum magazine_edge_type {
  E_MAGAZINE_EDGE_TYPE_RISING = 0,
  E_MAGAZINE_EDGE_TYPE_FALLING
};

struct probe_window {
  int32_t inboard;
  int32_t outboard;
  bool direction;
};

static struct {
  int32_t delta_pos_limit;
  bool on_probe;
} mag_state;

struct edge_event {
  bool state;
  int32_t position;
};

DECLARE_QUEUE(edge_events, struct edge_event, MAX_EDGE_EVENTS);


void magazine_init()
{
  // Set the magazine alignment position to the current position
  memcpy(sys.probe_position, sys.position, sizeof(int32_t) * N_AXIS);

  queue_init(&edge_events, sizeof(struct edge_event), MAX_EDGE_EVENTS);
  mag_state.delta_pos_limit = settings.mag_gap_limit * settings.steps_per_mm[C_AXIS];
}

void magazine_report_edge_events()
{
  while (!queue_is_empty(&edge_events)) {
    struct edge_event evt = {0};

    queue_dequeue(&edge_events, &evt);
    report_sensor_edge((uint8_t)MAG_SENSOR, evt.state, evt.position);
  }
}

static void magazine_edge_detector(const bool magazine_alignment_on)
{
  // We shouldn't send reports directly from here (as this is in an
  // interrupt context).  Instead, we queue them up and will send them
  // in the main program loop via a report request
  if (magazine_alignment_on != mag_state.on_probe) {
    struct edge_event evt = {0};
    evt.state = magazine_alignment_on;
    evt.position = sys.position[C_AXIS];

    queue_enqueue(&edge_events, &evt);
    request_report(REQUEST_EDGE_REPORT, 0);
  }

  mag_state.on_probe = magazine_alignment_on;
}

// Monitors the gap in units between mags and throws an alarm if the gap is larger than a
// specified threshold. Additionally, magazine slop can be calculated
// if turned on via compiler flag
void magazine_gap_monitor()
{
  const bool magazine_alignment_on = magazine_get_state();

  // When the probe is detected, copy the current carousel position
  // into the probe position
  if (magazine_alignment_on) {
    sys.probe_position[C_AXIS] = sys.position[C_AXIS];
  }

  magazine_edge_detector(magazine_alignment_on);

  if (limits.mag_gap_check == 0) {
    return;
  }

  // Activate alarm if the gap between the current position and the previous
  // probe position becomes too large. Only do this when the system has already homed
  const int32_t cur_pos = sys.position[C_AXIS];
  const int32_t probe_pos = sys.probe_position[C_AXIS];
  const int32_t delta_pos = abs(cur_pos - probe_pos);

  if (delta_pos > mag_state.delta_pos_limit) {
    sys.state = STATE_ALARM;
    sys.alarm |= ALARM_MAG_MISSING;
    SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
    st_go_idle();
    protocol_execute_runtime();
  }

}

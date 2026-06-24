#include "system.h"
#include "ejector_stepper.h"
#include "print.h"
#include "report.h"

#ifdef KEYME_J101_EJECTOR

#define EJECTOR_TIMER_HZ 2000000UL
#define EJECTOR_MIN_TIMER_TICKS 32U
#define EJECTOR_MAX_TIMER_TICKS 65535U
#define EJECTOR_UPDATE_INTERVAL_MS 100UL

#define EJECTOR_RESULT_SUCCESS 0U
#define EJECTOR_RESULT_FAILED 1U
#define EJECTOR_RESULT_HOME_TIMEOUT 2U
#define EJECTOR_RESULT_STOPPED 3U
#define EJECTOR_RESULT_PULLOFF_FAILED 4U
#define EJECTOR_RESULT_CONFIG_INVALID 5U

#define EJECTOR_HOME_PHASE_IDLE 0U
#define EJECTOR_HOME_PHASE_SEARCH 1U
#define EJECTOR_HOME_PHASE_PULL_OFF 2U
#define EJECTOR_HOME_PHASE_LOCATE 3U

typedef struct {
  bool active;
  bool homing;
  bool homed;
  bool configured;
  bool hold_enabled;
  bool direction_positive;
  bool direction_inverted;
  bool step_high;
  bool pending_update;
  bool pending_result;
  bool pending_continue;
  bool pending_timer_start;
  uint8_t pending_status;
  uint8_t home_phase;
  uint16_t position;
  uint16_t target;
  uint16_t final_target;
  uint16_t steps_remaining;
  uint16_t home_steps;
  uint16_t homing_phase_steps;
  uint16_t home_active_samples;
  uint16_t home_inactive_samples;
  uint16_t home_max_steps;
  uint16_t max_position;
  uint16_t home_switch_position;
  uint16_t rate_steps_per_second;
  uint16_t start_rate_steps_per_second;
  uint16_t current_rate_steps_per_second;
  uint16_t accel_steps_per_second2;
  uint16_t enable_settle_ms;
  uint16_t home_rate_steps_per_second;
  uint16_t home_start_rate_steps_per_second;
  uint16_t home_accel_steps_per_second2;
  uint16_t home_debounce_ms;
  uint16_t home_pull_off_steps;
  uint16_t home_locate_rate_steps_per_second;
  uint32_t current_rate_q8;
  uint32_t timer_start_ms;
} ejector_stepper_state_t;

static volatile ejector_stepper_state_t ejector_stepper;
static uint32_t last_update_ms;

static void schedule_result(uint8_t status);

static bool parse_uint16(char *line, uint8_t *char_counter, uint16_t *value)
{
  float parsed;

  if (!read_float(line, char_counter, &parsed)) {
    return false;
  }
  if ((parsed < 0.0) || (parsed > 65535.0)) {
    return false;
  }

  *value = (uint16_t)parsed;
  return true;
}

static uint8_t parse_comma(char *line, uint8_t *char_counter)
{
  if (line[*char_counter] != ',') {
    return STATUS_INVALID_STATEMENT;
  }
  (*char_counter)++;
  return STATUS_OK;
}

static void set_step_pin(bool high)
{
  if (high) {
    EJECTOR_STEPPER_PORT |= bit(EJECTOR_STEPPER_STEP_BIT);
  } else {
    EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_STEP_BIT);
  }
  ejector_stepper.step_high = high;
}

static void set_driver_enabled(bool enabled)
{
  if (enabled) {
    EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_ENABLE_N_BIT);
  } else {
    EJECTOR_STEPPER_PORT |= bit(EJECTOR_STEPPER_ENABLE_N_BIT);
  }
}

static void set_hold_enabled(bool enabled)
{
  ejector_stepper.hold_enabled = enabled;
  if (!enabled && ejector_stepper.active) {
    schedule_result(EJECTOR_RESULT_STOPPED);
  }
  set_driver_enabled(enabled);
}

static bool driver_enabled(void)
{
  return bit_isfalse(EJECTOR_STEPPER_PORT, bit(EJECTOR_STEPPER_ENABLE_N_BIT));
}

static bool home_switch_active(void)
{
  return bit_istrue(EJECTOR_STEPPER_PIN, bit(EJECTOR_STEPPER_HOME_BIT));
}

static void set_direction(bool positive)
{
  const bool drive_positive = positive != ejector_stepper.direction_inverted;

  ejector_stepper.direction_positive = positive;
  if (drive_positive) {
    EJECTOR_STEPPER_PORT |= bit(EJECTOR_STEPPER_DIR_BIT);
  } else {
    EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_DIR_BIT);
  }
}

static uint16_t timer_ticks(uint16_t rate)
{
  uint32_t ticks;

  if (rate == 0) {
    rate = 1U;
  }

  ticks = EJECTOR_TIMER_HZ / ((uint32_t)rate * 2UL);
  ticks = max(ticks, EJECTOR_MIN_TIMER_TICKS);
  ticks = min(ticks, EJECTOR_MAX_TIMER_TICKS);
  return (uint16_t)ticks;
}

static uint16_t target_rate(void)
{
  if (ejector_stepper.homing) {
    if (ejector_stepper.home_phase == EJECTOR_HOME_PHASE_LOCATE) {
      return ejector_stepper.home_locate_rate_steps_per_second;
    }
    return ejector_stepper.home_rate_steps_per_second;
  }
  return ejector_stepper.rate_steps_per_second;
}

static uint16_t start_rate(void)
{
  uint16_t rate;
  const uint16_t target = target_rate();

  if (ejector_stepper.homing) {
    rate = ejector_stepper.home_start_rate_steps_per_second;
  } else {
    rate = ejector_stepper.start_rate_steps_per_second;
  }

  if (rate > target) {
    rate = target;
  }
  return rate;
}

static uint16_t accel_rate(void)
{
  if (ejector_stepper.homing) {
    return ejector_stepper.home_accel_steps_per_second2;
  }
  return ejector_stepper.accel_steps_per_second2;
}

static uint32_t rate_delta_q8_per_step(uint16_t rate)
{
  uint32_t delta;

  if (rate == 0) {
    rate = start_rate();
  }

  delta = ((uint32_t)accel_rate() * 256UL) / (uint32_t)rate;
  return delta ? delta : 1UL;
}

static uint32_t steps_to_stop(void)
{
  const uint16_t start = start_rate();
  const uint16_t accel = accel_rate();
  const uint16_t current = ejector_stepper.current_rate_steps_per_second;
  uint32_t current_sq;
  uint32_t start_sq;

  if (current <= start) {
    return 0;
  }

  current_sq = (uint32_t)current * (uint32_t)current;
  start_sq = (uint32_t)start * (uint32_t)start;
  return (current_sq - start_sq) / (2UL * (uint32_t)accel);
}

static void update_timer_rate_q8(uint32_t rate_q8)
{
  uint16_t rate = (uint16_t)((rate_q8 + 128UL) / 256UL);

  if (rate == 0) {
    rate = start_rate();
    rate_q8 = (uint32_t)rate << 8;
  }

  ejector_stepper.current_rate_q8 = rate_q8;
  ejector_stepper.current_rate_steps_per_second = rate;
  OCR5A = timer_ticks(rate);
}

static void update_profile_rate(void)
{
  uint16_t rate;
  uint32_t rate_q8;
  uint32_t delta_q8;
  const uint16_t start = start_rate();
  const uint16_t target = target_rate();
  const uint32_t start_q8 = (uint32_t)start << 8;
  const uint32_t target_q8 = (uint32_t)target << 8;

  if (!ejector_stepper.active) {
    return;
  }

  rate_q8 = ejector_stepper.current_rate_q8 ?
    ejector_stepper.current_rate_q8 : start_q8;
  rate = (uint16_t)((rate_q8 + 128UL) / 256UL);
  delta_q8 = rate_delta_q8_per_step(rate);

  if (!ejector_stepper.homing &&
      ((uint32_t)ejector_stepper.steps_remaining <= steps_to_stop())) {
    rate_q8 = (rate_q8 > (start_q8 + delta_q8)) ?
      (rate_q8 - delta_q8) : start_q8;
  } else if (rate_q8 < target_q8) {
    rate_q8 = ((target_q8 - rate_q8) > delta_q8) ?
      (rate_q8 + delta_q8) : target_q8;
  }

  update_timer_rate_q8(rate_q8);
}

static void stop_timer(void)
{
  TIMSK5 &= ~bit(OCIE5A);
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  set_step_pin(false);
}

static void reset_profile_rate(void)
{
  const uint16_t rate = start_rate();

  ejector_stepper.current_rate_steps_per_second = rate;
  ejector_stepper.current_rate_q8 = (uint32_t)rate << 8;
  OCR5A = timer_ticks(rate);
}

static void start_timer(void)
{
  stop_timer();
  ejector_stepper.pending_timer_start = false;
  reset_profile_rate();
  TCCR5A = 0;
  TCCR5B = bit(WGM52) | bit(CS51);
  TIMSK5 |= bit(OCIE5A);
}

static void schedule_timer_start(void)
{
  stop_timer();
  reset_profile_rate();
  if (ejector_stepper.enable_settle_ms == 0) {
    start_timer();
    return;
  }

  ejector_stepper.pending_timer_start = true;
  ejector_stepper.timer_start_ms =
    masterclock + (uint32_t)ejector_stepper.enable_settle_ms;
}

static void snapshot(uint16_t *position,
                     bool *homed,
                     bool *active,
                     bool *homing,
                     bool *enabled,
                     bool *home_active)
{
  const uint8_t sreg = SREG;
  cli();
  *position = ejector_stepper.position;
  *homed = ejector_stepper.homed;
  *active = ejector_stepper.active;
  *homing = ejector_stepper.homing;
  *enabled = driver_enabled();
  *home_active = home_switch_active();
  SREG = sreg;
}

static void print_status_report(void)
{
  uint16_t position;
  bool homed;
  bool active;
  bool homing;
  bool enabled;
  bool home_active;

  snapshot(&position, &homed, &active, &homing, &enabled, &home_active);

  printPgmString(PSTR("[QEJ:"));
  printInteger(position);
  printPgmString(PSTR(","));
  print_uint8_base10(homed ? 1 : 0);
  printPgmString(PSTR(","));
  print_uint8_base10(active ? 1 : 0);
  printPgmString(PSTR(","));
  print_uint8_base10(homing ? 1 : 0);
  printPgmString(PSTR(","));
  print_uint8_base10(enabled ? 1 : 0);
  printPgmString(PSTR(","));
  print_uint8_base10(home_active ? 1 : 0);
  printPgmString(PSTR(","));
  print_uint8_base10(ejector_stepper.home_phase);
  printPgmString(PSTR(","));
  printInteger(ejector_stepper.current_rate_steps_per_second);
  printPgmString(PSTR(","));
  printInteger(ejector_stepper.target);
  printPgmString(PSTR(","));
  print_uint8_base10(ejector_stepper.pending_status);
  printPgmString(PSTR("]\r\n"));
}

static void print_result_report(uint8_t status, uint16_t position)
{
  printPgmString(PSTR("[QER:"));
  print_uint8_base10(status);
  printPgmString(PSTR(","));
  printInteger(position);
  printPgmString(PSTR("]\r\n"));
}

static void schedule_result(uint8_t status)
{
  ejector_stepper.active = false;
  ejector_stepper.homing = false;
  ejector_stepper.pending_continue = false;
  ejector_stepper.pending_timer_start = false;
  ejector_stepper.pending_update = true;
  ejector_stepper.pending_result = true;
  ejector_stepper.pending_status = status;
  ejector_stepper.home_phase = EJECTOR_HOME_PHASE_IDLE;
  stop_timer();
}

static void begin_counted_move(uint16_t target)
{
  uint16_t delta;

  if (!ejector_stepper.configured) {
    schedule_result(EJECTOR_RESULT_CONFIG_INVALID);
    return;
  }
  if (target > ejector_stepper.max_position) {
    schedule_result(EJECTOR_RESULT_FAILED);
    return;
  }
  if (!driver_enabled()) {
    schedule_result(EJECTOR_RESULT_FAILED);
    return;
  }

  ejector_stepper.target = target;
  if (target == ejector_stepper.position) {
    schedule_result(EJECTOR_RESULT_SUCCESS);
    return;
  }

  if (target > ejector_stepper.position) {
    delta = target - ejector_stepper.position;
    set_direction(true);
  } else {
    delta = ejector_stepper.position - target;
    set_direction(false);
  }

  ejector_stepper.steps_remaining = delta;
  ejector_stepper.homing = false;
  ejector_stepper.home_phase = EJECTOR_HOME_PHASE_IDLE;
  ejector_stepper.active = true;
  ejector_stepper.pending_update = true;
  schedule_timer_start();
}

static uint16_t home_debounce_steps(void)
{
  uint32_t steps = ((uint32_t)target_rate() *
    (uint32_t)ejector_stepper.home_debounce_ms + 999UL) / 1000UL;

  if (steps == 0) {
    steps = 1;
  }
  return (uint16_t)min(steps, 65535UL);
}

static void begin_homing_phase(uint8_t phase, bool direction_positive)
{
  ejector_stepper.home_phase = phase;
  ejector_stepper.homing_phase_steps = 0;
  ejector_stepper.home_active_samples = 0;
  ejector_stepper.home_inactive_samples = 0;
  ejector_stepper.homing = true;
  ejector_stepper.active = true;
  ejector_stepper.pending_update = true;
  set_direction(direction_positive);
  reset_profile_rate();
}

static void start_homing(uint16_t final_target)
{
  if (!ejector_stepper.configured) {
    schedule_result(EJECTOR_RESULT_CONFIG_INVALID);
    return;
  }

  ejector_stepper.final_target = final_target;
  ejector_stepper.target = ejector_stepper.home_switch_position;
  ejector_stepper.home_steps = 0;
  if (!driver_enabled()) {
    schedule_result(EJECTOR_RESULT_FAILED);
    return;
  }
  ejector_stepper.homing = true;
  ejector_stepper.active = true;
  if (home_switch_active()) {
    begin_homing_phase(EJECTOR_HOME_PHASE_PULL_OFF, true);
  } else {
    begin_homing_phase(EJECTOR_HOME_PHASE_SEARCH, false);
  }
  schedule_timer_start();
}

static void go_to_target(uint16_t target)
{
  if (!ejector_stepper.homed) {
    start_homing(target);
    return;
  }

  begin_counted_move(target);
}

static void stop_motion(uint8_t status)
{
  if (ejector_stepper.active) {
    schedule_result(status);
  } else {
    stop_timer();
  }
}

static void complete_home(void)
{
  ejector_stepper.position = ejector_stepper.home_switch_position;
  ejector_stepper.homed = true;
  ejector_stepper.homing = false;
  ejector_stepper.active = false;
  ejector_stepper.pending_timer_start = false;
  ejector_stepper.home_phase = EJECTOR_HOME_PHASE_IDLE;
  stop_timer();

  if (ejector_stepper.final_target == ejector_stepper.home_switch_position) {
    schedule_result(EJECTOR_RESULT_SUCCESS);
  } else {
    ejector_stepper.pending_update = true;
    ejector_stepper.pending_continue = true;
  }
}

static void handle_rising_edge(void)
{
  if (ejector_stepper.homing) {
    const bool home_active = home_switch_active();
    const uint16_t debounce_steps = home_debounce_steps();

    ejector_stepper.home_steps++;
    ejector_stepper.homing_phase_steps++;

    switch (ejector_stepper.home_phase) {
      case EJECTOR_HOME_PHASE_SEARCH:
        if (home_active) {
          ejector_stepper.home_active_samples++;
          if (ejector_stepper.home_active_samples >= debounce_steps) {
            begin_homing_phase(EJECTOR_HOME_PHASE_PULL_OFF, true);
          }
        } else {
          ejector_stepper.home_active_samples = 0;
        }
        if (ejector_stepper.active &&
            (ejector_stepper.homing_phase_steps >=
             ejector_stepper.home_max_steps)) {
          ejector_stepper.homed = false;
          schedule_result(EJECTOR_RESULT_HOME_TIMEOUT);
        }
        break;

      case EJECTOR_HOME_PHASE_PULL_OFF:
        if (home_active) {
          ejector_stepper.home_inactive_samples = 0;
        } else {
          ejector_stepper.home_inactive_samples++;
        }
        if ((ejector_stepper.home_inactive_samples >= debounce_steps) &&
            (ejector_stepper.homing_phase_steps >=
             ejector_stepper.home_pull_off_steps)) {
          begin_homing_phase(EJECTOR_HOME_PHASE_LOCATE, false);
        } else if (ejector_stepper.homing_phase_steps >=
                   ejector_stepper.home_max_steps) {
          ejector_stepper.homed = false;
          schedule_result(EJECTOR_RESULT_PULLOFF_FAILED);
        }
        break;

      case EJECTOR_HOME_PHASE_LOCATE:
        if (home_active) {
          ejector_stepper.home_active_samples++;
          if (ejector_stepper.home_active_samples >= debounce_steps) {
            complete_home();
          }
        } else {
          ejector_stepper.home_active_samples = 0;
        }
        if (ejector_stepper.active &&
            (ejector_stepper.homing_phase_steps >=
             ejector_stepper.home_max_steps)) {
          ejector_stepper.homed = false;
          schedule_result(EJECTOR_RESULT_HOME_TIMEOUT);
        }
        break;

      default:
        ejector_stepper.homed = false;
        schedule_result(EJECTOR_RESULT_FAILED);
        break;
    }
    return;
  }

  if (ejector_stepper.direction_positive) {
    if (ejector_stepper.position < ejector_stepper.max_position) {
      ejector_stepper.position++;
    }
  } else if (ejector_stepper.position > 0) {
    ejector_stepper.position--;
  }

  if (ejector_stepper.steps_remaining > 0) {
    ejector_stepper.steps_remaining--;
  }
  if (ejector_stepper.steps_remaining == 0) {
    schedule_result(EJECTOR_RESULT_SUCCESS);
  }
}

ISR(TIMER5_COMPA_vect)
{
  if (!ejector_stepper.active) {
    stop_timer();
    return;
  }

  if (ejector_stepper.step_high) {
    set_step_pin(false);
    return;
  }

  set_step_pin(true);
  handle_rising_edge();
  update_profile_rate();
}

void ejector_stepper_init(void)
{
  stop_timer();

  EJECTOR_STEPPER_DDR |= bit(EJECTOR_STEPPER_STEP_BIT);
  EJECTOR_STEPPER_DDR |= bit(EJECTOR_STEPPER_DIR_BIT);
  EJECTOR_STEPPER_DDR |= bit(EJECTOR_STEPPER_ENABLE_N_BIT);
  EJECTOR_STEPPER_DDR &= ~bit(EJECTOR_STEPPER_HOME_BIT);

  EJECTOR_STEPPER_PORT |= bit(EJECTOR_STEPPER_HOME_BIT);
  EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_STEP_BIT);
  EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_DIR_BIT);
  EJECTOR_STEPPER_PORT &= ~bit(EJECTOR_STEPPER_ENABLE_N_BIT);

  ejector_stepper.active = false;
  ejector_stepper.homing = false;
  ejector_stepper.homed = false;
  ejector_stepper.configured = false;
  ejector_stepper.hold_enabled = true;
  ejector_stepper.direction_positive = true;
  ejector_stepper.direction_inverted = false;
  ejector_stepper.step_high = false;
  ejector_stepper.pending_update = false;
  ejector_stepper.pending_result = false;
  ejector_stepper.pending_continue = false;
  ejector_stepper.pending_timer_start = false;
  ejector_stepper.pending_status = EJECTOR_RESULT_SUCCESS;
  ejector_stepper.home_phase = EJECTOR_HOME_PHASE_IDLE;
  ejector_stepper.position = 0;
  ejector_stepper.target = 0;
  ejector_stepper.final_target = 0;
  ejector_stepper.steps_remaining = 0;
  ejector_stepper.home_steps = 0;
  ejector_stepper.homing_phase_steps = 0;
  ejector_stepper.home_active_samples = 0;
  ejector_stepper.home_inactive_samples = 0;
  ejector_stepper.home_max_steps = 0;
  ejector_stepper.max_position = 0;
  ejector_stepper.home_switch_position = 0;
  ejector_stepper.rate_steps_per_second = 0;
  ejector_stepper.start_rate_steps_per_second = 0;
  ejector_stepper.current_rate_steps_per_second = 0;
  ejector_stepper.accel_steps_per_second2 = 0;
  ejector_stepper.enable_settle_ms = 0;
  ejector_stepper.home_rate_steps_per_second = 0;
  ejector_stepper.home_start_rate_steps_per_second = 0;
  ejector_stepper.home_accel_steps_per_second2 = 0;
  ejector_stepper.home_debounce_ms = 0;
  ejector_stepper.home_pull_off_steps = 0;
  ejector_stepper.home_locate_rate_steps_per_second = 0;
  ejector_stepper.current_rate_q8 = 0;
  ejector_stepper.timer_start_ms = 0;
  last_update_ms = masterclock;
}

void ejector_stepper_service(void)
{
  uint16_t result_position;
  uint8_t result_status;

  if (ejector_stepper.pending_continue) {
    ejector_stepper.pending_continue = false;
    begin_counted_move(ejector_stepper.final_target);
  }

  if (ejector_stepper.pending_timer_start &&
      ((int32_t)(masterclock - ejector_stepper.timer_start_ms) >= 0)) {
    start_timer();
  }

  if (ejector_stepper.active &&
      ((uint32_t)(masterclock - last_update_ms) >= EJECTOR_UPDATE_INTERVAL_MS)) {
    last_update_ms = masterclock;
    print_status_report();
  }

  if (ejector_stepper.pending_update) {
    ejector_stepper.pending_update = false;
    print_status_report();
  }

  if (ejector_stepper.pending_result) {
    const uint8_t sreg = SREG;
    cli();
    result_status = ejector_stepper.pending_status;
    result_position = ejector_stepper.position;
    ejector_stepper.pending_result = false;
    SREG = sreg;
    print_result_report(result_status, result_position);
  }
}

bool ejector_stepper_is_active(void)
{
  return ejector_stepper.active;
}

uint8_t ejector_stepper_execute_line(char *line, uint8_t *char_counter)
{
  uint16_t rate;
  uint16_t home_max;
  uint16_t max_position;
  uint16_t home_switch_position;
  uint16_t direction_inverted;
  uint16_t start_rate_config;
  uint16_t accel_config;
  uint16_t enable_settle_ms;
  uint16_t home_rate;
  uint16_t home_start_rate;
  uint16_t home_accel;
  uint16_t home_debounce_ms;
  uint16_t home_pull_off_steps;
  uint16_t home_locate_rate;
  uint16_t target;
  uint8_t status;

  (*char_counter)++;

  switch (line[*char_counter]) {
    case 'C':
      (*char_counter)++;
      if (ejector_stepper.active) {
        return STATUS_IDLE_ERROR;
      }
      if (!parse_uint16(line, char_counter, &rate)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_max)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &max_position)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_switch_position)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &direction_inverted)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &start_rate_config)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &accel_config)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &enable_settle_ms)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_rate)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_start_rate)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_accel)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_debounce_ms)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_pull_off_steps)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      status = parse_comma(line, char_counter);
      if (status) {
        return status;
      }
      if (!parse_uint16(line, char_counter, &home_locate_rate)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      if ((rate == 0) || (home_max == 0) || (max_position == 0) ||
          (start_rate_config == 0) || (accel_config == 0) ||
          (home_pull_off_steps == 0) || (home_rate == 0) ||
          (home_start_rate == 0) || (home_accel == 0) ||
          (home_locate_rate == 0) ||
          (home_switch_position > max_position)) {
        return STATUS_INVALID_STATEMENT;
      }
      ejector_stepper.rate_steps_per_second = rate;
      ejector_stepper.home_max_steps = home_max;
      ejector_stepper.max_position = max_position;
      ejector_stepper.home_switch_position = home_switch_position;
      ejector_stepper.direction_inverted = direction_inverted != 0;
      ejector_stepper.start_rate_steps_per_second = start_rate_config;
      ejector_stepper.accel_steps_per_second2 = accel_config;
      ejector_stepper.enable_settle_ms = enable_settle_ms;
      ejector_stepper.home_rate_steps_per_second = home_rate;
      ejector_stepper.home_start_rate_steps_per_second = home_start_rate;
      ejector_stepper.home_accel_steps_per_second2 = home_accel;
      ejector_stepper.home_debounce_ms = home_debounce_ms;
      ejector_stepper.home_pull_off_steps = home_pull_off_steps;
      ejector_stepper.home_locate_rate_steps_per_second = home_locate_rate;
      ejector_stepper.configured = true;
      return STATUS_OK;

    case 'H':
      (*char_counter)++;
      if (line[*char_counter] == 0) {
        target = ejector_stepper.home_switch_position;
      } else if (!parse_uint16(line, char_counter, &target)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      if (!ejector_stepper.configured) {
        schedule_result(EJECTOR_RESULT_CONFIG_INVALID);
        return STATUS_OK;
      }
      if (target > ejector_stepper.max_position) {
        return STATUS_INVALID_STATEMENT;
      }
      if (ejector_stepper.active) {
        return STATUS_IDLE_ERROR;
      }
      start_homing(target);
      return STATUS_OK;

    case 'M':
      (*char_counter)++;
      if (!parse_uint16(line, char_counter, &target)) {
        return STATUS_BAD_NUMBER_FORMAT;
      }
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      if (!ejector_stepper.configured) {
        schedule_result(EJECTOR_RESULT_CONFIG_INVALID);
        return STATUS_OK;
      }
      if (target > ejector_stepper.max_position) {
        return STATUS_INVALID_STATEMENT;
      }
      if (ejector_stepper.active) {
        return STATUS_IDLE_ERROR;
      }
      go_to_target(target);
      return STATUS_OK;

    case 'S':
      (*char_counter)++;
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      stop_motion(EJECTOR_RESULT_STOPPED);
      return STATUS_OK;

    case 'E':
      (*char_counter)++;
      if (line[*char_counter] == '0') {
        set_hold_enabled(false);
      } else if (line[*char_counter] == '1') {
        set_hold_enabled(true);
      } else {
        return STATUS_INVALID_STATEMENT;
      }
      (*char_counter)++;
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      return STATUS_OK;

    case 'P':
      (*char_counter)++;
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      print_status_report();
      return STATUS_QUIET_OK;

    default:
      return STATUS_INVALID_STATEMENT;
  }
}

#else

void ejector_stepper_init(void) {}
void ejector_stepper_service(void) {}
bool ejector_stepper_is_active(void) { return false; }
uint8_t ejector_stepper_execute_line(char *line, uint8_t *char_counter)
{
  UNUSED(line);
  UNUSED(char_counter);
  return STATUS_INVALID_STATEMENT;
}

#endif

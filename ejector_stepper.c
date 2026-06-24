#include "system.h"
#include "ejector_stepper.h"
#include "print.h"
#include "report.h"

#ifdef KEYME_J101_EJECTOR

#define EJECTOR_DEFAULT_RATE_STEPS_PER_SECOND 800U
#define EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND 50U
#define EJECTOR_DEFAULT_ACCEL_STEPS_PER_SECOND2 500U
#define EJECTOR_DEFAULT_HOME_MAX_STEPS 1200U
#define EJECTOR_DEFAULT_MAX_POSITION 65534U
#define EJECTOR_TIMER_HZ 2000000UL
#define EJECTOR_MIN_TIMER_TICKS 32U
#define EJECTOR_MAX_TIMER_TICKS 65535U
#define EJECTOR_UPDATE_INTERVAL_MS 100UL

#define EJECTOR_RESULT_SUCCESS 0U
#define EJECTOR_RESULT_FAILED 1U
#define EJECTOR_RESULT_STOPPED 3U

typedef struct {
  bool active;
  bool homing;
  bool homed;
  bool hold_enabled;
  bool direction_positive;
  bool direction_inverted;
  bool step_high;
  bool pending_update;
  bool pending_result;
  bool pending_continue;
  uint8_t pending_status;
  uint16_t position;
  uint16_t target;
  uint16_t final_target;
  uint16_t steps_remaining;
  uint16_t home_steps;
  uint16_t home_max_steps;
  uint16_t max_position;
  uint16_t home_switch_position;
  uint16_t rate_steps_per_second;
  uint16_t start_rate_steps_per_second;
  uint16_t current_rate_steps_per_second;
  uint16_t accel_steps_per_second2;
} ejector_stepper_state_t;

static volatile ejector_stepper_state_t ejector_stepper;
static uint32_t last_update_ms;

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
  set_driver_enabled(enabled || ejector_stepper.active);
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
    rate = EJECTOR_DEFAULT_RATE_STEPS_PER_SECOND;
  }

  ticks = EJECTOR_TIMER_HZ / ((uint32_t)rate * 2UL);
  ticks = max(ticks, EJECTOR_MIN_TIMER_TICKS);
  ticks = min(ticks, EJECTOR_MAX_TIMER_TICKS);
  return (uint16_t)ticks;
}

static uint16_t target_rate(void)
{
  return ejector_stepper.rate_steps_per_second ?
    ejector_stepper.rate_steps_per_second :
    EJECTOR_DEFAULT_RATE_STEPS_PER_SECOND;
}

static uint16_t start_rate(void)
{
  uint16_t rate = ejector_stepper.start_rate_steps_per_second ?
    ejector_stepper.start_rate_steps_per_second :
    EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND;
  const uint16_t target = target_rate();

  if (rate > target) {
    rate = target;
  }
  return rate;
}

static uint16_t accel_rate(void)
{
  return ejector_stepper.accel_steps_per_second2 ?
    ejector_stepper.accel_steps_per_second2 :
    EJECTOR_DEFAULT_ACCEL_STEPS_PER_SECOND2;
}

static uint16_t rate_delta_per_step(uint16_t rate)
{
  uint16_t delta;

  if (rate == 0) {
    rate = start_rate();
  }

  delta = accel_rate() / rate;
  return delta ? delta : 1U;
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

static void update_timer_rate(uint16_t rate)
{
  ejector_stepper.current_rate_steps_per_second = rate;
  OCR5A = timer_ticks(rate);
}

static void update_profile_rate(void)
{
  uint16_t rate;
  uint16_t delta;
  const uint16_t start = start_rate();
  const uint16_t target = target_rate();

  if (!ejector_stepper.active) {
    return;
  }

  rate = ejector_stepper.current_rate_steps_per_second;
  if (rate == 0) {
    rate = start;
  }
  delta = rate_delta_per_step(rate);

  if (!ejector_stepper.homing &&
      ((uint32_t)ejector_stepper.steps_remaining <= steps_to_stop())) {
    rate = ((uint16_t)(rate - start) > delta) ? (rate - delta) : start;
  } else if (rate < target) {
    rate = ((uint16_t)(target - rate) > delta) ? (rate + delta) : target;
  }

  update_timer_rate(rate);
}

static void stop_timer(void)
{
  TIMSK5 &= ~bit(OCIE5A);
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  set_step_pin(false);
}

static void start_timer(void)
{
  stop_timer();
  ejector_stepper.current_rate_steps_per_second = start_rate();
  OCR5A = timer_ticks(ejector_stepper.current_rate_steps_per_second);
  TCCR5A = 0;
  TCCR5B = bit(WGM52) | bit(CS51);
  TIMSK5 |= bit(OCIE5A);
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
  ejector_stepper.pending_update = true;
  ejector_stepper.pending_result = true;
  ejector_stepper.pending_status = status;
  stop_timer();
  set_driver_enabled(ejector_stepper.hold_enabled);
}

static void begin_counted_move(uint16_t target)
{
  uint16_t delta;

  if (target > ejector_stepper.max_position) {
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
  ejector_stepper.active = true;
  ejector_stepper.pending_update = true;
  set_driver_enabled(true);
  start_timer();
}

static void start_homing(uint16_t final_target)
{
  ejector_stepper.final_target = final_target;
  ejector_stepper.target = ejector_stepper.home_switch_position;
  ejector_stepper.home_steps = 0;

  if (home_switch_active()) {
    ejector_stepper.position = ejector_stepper.home_switch_position;
    ejector_stepper.homed = true;
    if (final_target == ejector_stepper.home_switch_position) {
      schedule_result(EJECTOR_RESULT_SUCCESS);
    } else {
      begin_counted_move(final_target);
    }
    return;
  }

  set_direction(false);
  ejector_stepper.homing = true;
  ejector_stepper.active = true;
  ejector_stepper.pending_update = true;
  set_driver_enabled(true);
  start_timer();
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
    set_driver_enabled(ejector_stepper.hold_enabled);
  }
}

static void complete_home(void)
{
  ejector_stepper.position = ejector_stepper.home_switch_position;
  ejector_stepper.homed = true;
  ejector_stepper.homing = false;
  ejector_stepper.active = false;
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
    ejector_stepper.home_steps++;
    if (home_switch_active()) {
      complete_home();
    } else if (ejector_stepper.home_steps >= ejector_stepper.home_max_steps) {
      ejector_stepper.homed = false;
      schedule_result(EJECTOR_RESULT_FAILED);
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
  EJECTOR_STEPPER_PORT |= bit(EJECTOR_STEPPER_ENABLE_N_BIT);

  ejector_stepper.active = false;
  ejector_stepper.homing = false;
  ejector_stepper.homed = false;
  ejector_stepper.hold_enabled = false;
  ejector_stepper.direction_positive = true;
  ejector_stepper.direction_inverted = false;
  ejector_stepper.step_high = false;
  ejector_stepper.pending_update = false;
  ejector_stepper.pending_result = false;
  ejector_stepper.pending_continue = false;
  ejector_stepper.pending_status = EJECTOR_RESULT_SUCCESS;
  ejector_stepper.position = 0;
  ejector_stepper.target = 0;
  ejector_stepper.final_target = 0;
  ejector_stepper.steps_remaining = 0;
  ejector_stepper.home_steps = 0;
  ejector_stepper.home_max_steps = EJECTOR_DEFAULT_HOME_MAX_STEPS;
  ejector_stepper.max_position = EJECTOR_DEFAULT_MAX_POSITION;
  ejector_stepper.home_switch_position = 0;
  ejector_stepper.rate_steps_per_second = EJECTOR_DEFAULT_RATE_STEPS_PER_SECOND;
  ejector_stepper.start_rate_steps_per_second =
    EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND;
  ejector_stepper.current_rate_steps_per_second =
    EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND;
  ejector_stepper.accel_steps_per_second2 =
    EJECTOR_DEFAULT_ACCEL_STEPS_PER_SECOND2;
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
      start_rate_config = EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND;
      accel_config = EJECTOR_DEFAULT_ACCEL_STEPS_PER_SECOND2;
      if (line[*char_counter] == ',') {
        (*char_counter)++;
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
      }
      if (line[*char_counter] != 0) {
        return STATUS_INVALID_STATEMENT;
      }
      ejector_stepper.rate_steps_per_second =
        rate ? rate : EJECTOR_DEFAULT_RATE_STEPS_PER_SECOND;
      ejector_stepper.home_max_steps =
        home_max ? home_max : EJECTOR_DEFAULT_HOME_MAX_STEPS;
      ejector_stepper.max_position =
        max_position ? max_position : EJECTOR_DEFAULT_MAX_POSITION;
      ejector_stepper.home_switch_position = home_switch_position;
      ejector_stepper.direction_inverted = direction_inverted != 0;
      ejector_stepper.start_rate_steps_per_second =
        start_rate_config ? start_rate_config :
        EJECTOR_DEFAULT_START_RATE_STEPS_PER_SECOND;
      ejector_stepper.accel_steps_per_second2 =
        accel_config ? accel_config : EJECTOR_DEFAULT_ACCEL_STEPS_PER_SECOND2;
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

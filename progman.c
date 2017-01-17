#include "gqueue.h"

#include "system.h"
#include "report.h"
#include "serial.h"
#include "motion_control.h"

#define PROGRAM_MAX_SIZE (1024 * 3)

DECLARE_QUEUE(prog_buf, uint8_t, PROGRAM_MAX_SIZE);

enum progman_state {
  E_PROGMAN_CONSUMING = 0,
  E_PROGMAN_COMMENT
};

static struct {
  uint8_t line_assy[LINE_BUFFER_SIZE];
  uint8_t line_pos;
  bool in_comment;
  enum progman_state state;
} progman;

void progman_init(void)
{
  progman.line_pos = 0;

  queue_init(&prog_buf, sizeof(uint8_t), PROGRAM_MAX_SIZE);

  while (!queue_is_empty(&prog_buf)) {
    uint8_t nil;
    queue_dequeue(&prog_buf, &nil);
  }
  progman.state = E_PROGMAN_CONSUMING;
}

bool progman_handle_runtime_cmd(char data)
{
  switch (data) {
  case CMD_COUNTER_REPORT:
    request_report(REQUEST_COUNTER_REPORT,0);
    break;
  case CMD_VOLTAGE_REPORT:
    request_report(REQUEST_VOLTAGE_REPORT,0);
    break;
  case CMD_STATUS_REPORT:
    request_report(REQUEST_STATUS_REPORT,0);
    break;
  case CMD_LIMIT_REPORT:
    request_report(REQUEST_LIMIT_REPORT,0);
    break;
  case CMD_EDGE_REPORT:
    request_report(REQUEST_EDGE_REPORT,0);
    break;
  case CMD_CYCLE_START:
    /* Set as true */
    SYS_EXEC |= EXEC_CYCLE_START;
    break;
  case CMD_FEED_HOLD:
    /* Set as true */
    SYS_EXEC |= EXEC_FEED_HOLD;
    break;
  case CMD_RESET:
    /* Call motion control reset routine. */
    mc_reset();
    break;
  case '\n':			/* Intentional fall through */
  case '\r':
    /* If we got a bare newline character out of context, it's likely
       due to the fact that our previous character was consumed as
       a runtime command, so we should just swallow it */
    if ((progman.line_pos < 1) &&
	(E_PROGMAN_CONSUMING == progman.state)) {
      report_status_message(STATUS_OK);
      break;
    }
  default:
    /* If we didn't handle the command, return false  */
    return false;
  }

  return true;
}

static bool progman_consume_byte(char data)
{

  /* Immediately try to toss out any comments */
  switch (data) {
  case '(':
    progman.state = E_PROGMAN_COMMENT;
    break;
  case ')':
    if (E_PROGMAN_COMMENT == progman.state) {
      progman.state = E_PROGMAN_CONSUMING;
    } else {
      report_status_message(STATUS_INVALID_STATEMENT);
    }
    break;
  }

  if (E_PROGMAN_COMMENT == progman.state) {
    return false;
  }

  /* Discard whitespace and control characters to maximize space
     efficiency */
  switch(data) {
  case ' ':
  case '/':
    /* TODO: Figure out how to handle block delete -JC */
    return false;
  }

  /* Store the data */
  progman.line_assy[progman.line_pos++] = data;

  /* If we're at the end of a line, return true so that the line will
     be validated */
  switch(data) {
  case '\r':
  case '\n':
    return true;
  }

  return false;
}

bool progman_validate_gcode_line(void)
{
  /* Filters out any line that contains non printable characters */
  int i;
  for (i = 0; i < progman.line_pos; i++) {
    /* Pass on whitespace */
    switch (progman.line_assy[i]) {
    case '\r':
    case '\n':
      continue;
    default:
      if (progman.line_assy[i] < ' ' || progman.line_assy[i] > '~') {
	return false;
      }
    }
  }
  return true;
}

static bool progman_buffer_gcode_line(void)
{
  /* Moves the currently assembled gcode line into the program buffer */
  int i;
  bool result = true;
  for (i = 0; i < progman.line_pos; i++) {
    if (queue_is_full(&prog_buf)) {
      result = false;
      goto cleanup_and_exit;
    }
    queue_enqueue(&prog_buf, &progman.line_assy[i]);
  }

cleanup_and_exit:
  progman.line_pos = 0;
  return result;
}

bool progman_read(uint8_t *dst)
{
  /* Attempts to move a single byte into *dst. Returns true for
     success, or false if the queue is empty */

  if (queue_is_empty(&prog_buf)) {
    return false;
  }

  queue_dequeue(&prog_buf, dst);
  return true;
}

void progman_execute(void)
{
  /* Just loop around if our buffer is full */
  if (queue_is_full(&prog_buf)) {
    return;
  }

  uint8_t c = serial_read();

  if (c == SERIAL_NO_DATA) {
    return;
  }

  /* Runtime commands are a special case, we short circuit putting
     them in the buffer and perform the specified action
     immediately */
  if (progman_handle_runtime_cmd(c)) {
    return;
  }

  /* Try to consume the byte (ignoring comments).  A successful
     return means we've consumed an entire line and should attempt to
     validate it. */
  if (progman_consume_byte(c)) {
    if (!progman_validate_gcode_line()) {
      report_status_message(STATUS_INVALID_STATEMENT);
      progman.line_pos = 0;
      return;
    }

    if (!progman_buffer_gcode_line()) {
      report_status_message(STATUS_OVERFLOW);
      return;
    }
    report_status_message(STATUS_OK);
  }
}

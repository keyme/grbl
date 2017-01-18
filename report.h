/*
  report.h - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon

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
#ifndef report_h
#define report_h

// Define Grbl status codes.
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define STATUS_INVALID_STATEMENT 3
#define STATUS_NEGATIVE_VALUE 4
#define STATUS_SETTING_DISABLED 5
#define STATUS_SETTING_STEP_PULSE_MIN 6
#define STATUS_SETTING_READ_FAIL 7
#define STATUS_IDLE_ERROR 8
#define STATUS_ALARM_LOCK 9
#define STATUS_SOFT_LIMIT_ERROR 10
#define STATUS_OVERFLOW 11
#define STATUS_QUIET_OK (1<<7)
#define STATUS_ALT_REPORT(rpt) (STATUS_QUIET_OK|rpt)

#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28
#define STATUS_GCODE_UNSUPPORTED_COORD_SYS 29
#define STATUS_GCODE_G53_INVALID_MOTION_MODE 30
#define STATUS_GCODE_AXIS_WORDS_EXIST 31
#define STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE 32
#define STATUS_GCODE_INVALID_TARGET 33
#define STATUS_GCODE_ARC_RADIUS_ERROR 34
#define STATUS_GCODE_NO_OFFSETS_IN_PLANE 35
#define STATUS_GCODE_PROBE_TRIGGERED 36
#define STATUS_GCODE_UNUSED_WORDS 37
#define STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR 38
#define STATUS_GCODE_NO_PROBE_SENSOR_SPECIFIED 39
#define STATUS_GCODE_NO_FAIL 40

// Define Grbl feedback message codes.
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5

// Prints system status messages.
void report_status_message(uint8_t status_code);

// Prints debug message which shows up in MOTION
void report_debug_message(const char *s);

// Prints system alarm messages.
void report_alarm_message(int8_t alarm_code);

// Prints miscellaneous feedback messages.
void report_feedback_message(uint8_t message_code);

// Prints welcome message
void report_init_message();

// Prints Grbl help and current global settings
void report_grbl_help();

// Prints Grbl global settings
void report_grbl_settings();

// Prints realtime status report
uint8_t report_realtime_status();

// Prints state of limit pins and estop
void report_limit_pins();

// Prints state of counters
void report_counters();

// Read voltage of ADCs

void report_voltage();
void calculate_motor_voltage();
void calculate_force_voltage();
void report_revision();

// Reporting of sensor edges
void report_sensor_edge(uint8_t sensor, bool state, int32_t axis_position);

// Prints recorded probe position
void report_probe_parameters(uint8_t error);

// Prints a message indicating probe failure
void report_probe_fail();

// Prints Grbl NGC parameters (coordinate offsets, probe)
void report_ngc_parameters();

// Prints current g-code parser mode state
void report_gcode_modes();

// Prints startup line
void report_startup_line(uint8_t n, char *line);

// Prints build info and user info
void report_build_info(char *line);

// Prints current limit word
void report_limit_pins();

#define request_report(report,exec) (sysflags.report_rqsts|=(report))&&(SYS_EXEC|=(EXEC_RUNTIME_REPORT|(exec)))
#define request_eol_report()  (sys.flags|=SYSFLAG_EOL_REPORT);request_report(REQUEST_STATUS_REPORT,0)

#endif

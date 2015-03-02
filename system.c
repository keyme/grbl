/*
  system.c - Handles system level commands and real-time processes
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

#include "system.h"
#include "settings.h"
#include "gcode.h"
#include "motion_control.h"
#include "report.h"
#include "print.h"
#include "counters.h"


uint32_t masterclock=0;  


void system_init() 
{
  TIMING_DDR |= TIMING_MASK;
  TIME_ON(ACTIVE_TIMER);

  linenumber_init();

  //setup masterclock
  
  TIMSK2 &= (~(1<<OCIE2B));  
  TIMSK2 |= ((1<<TOIE2)|(1<<OCIE2A)); // ovf INTERRUPT //TODO: test without TOIE2
  TCCR2A = (1<<WGM21);  // CTC Mode, use OCRA2 as top 
  TCCR2B = 4;           // 64 prescale
  OCR2A = 249;          //(249+1) * 64 prescale / 16Mhz = 1 ms
  OCR2B = 0;
}

ISR(TIMER2_COMPA_vect)
{
  TIME_TOGGLE(time_CLOCK);
  masterclock++;
}



// Executes user startup script, if stored.
void system_execute_startup(char *line) 
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      report_status_message(STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        printString(line); // Echo startup line to indicate execution.
        report_status_message(gc_execute_line(line));
      }
    } 
  }  
}

// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as 
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the runtime command module by being susceptible to when Grbl is ready to execute the 
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle, 
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t system_execute_line(char *line) 
{   
  uint8_t char_counter = 1; 
  uint8_t helper_var = 0; // Helper variable
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case 'G' : // Prints gcode parser state
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      else { report_gcode_modes();  return(STATUS_QUIET_OK);}
      break;   
    case 'C' : // Set check g-code mode [IDLE/CHECK]
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      // Perform reset when toggling off. Check g-code mode should only work if Grbl
      // is idle and ready, regardless of alarm locks. This is mainly to keep things
      // simple and consistent.
      if ( sys.state == STATE_CHECK_MODE ) { 
        mc_reset(); 
        report_feedback_message(MESSAGE_DISABLED);
      } else {
        if (sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
        sys.state = STATE_CHECK_MODE;
        report_feedback_message(MESSAGE_ENABLED);
      }
      break; 
    case 'X' : // Disable alarm lock [ALARM]
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      if (sys.state == STATE_ALARM) { 
        report_feedback_message(MESSAGE_ALARM_UNLOCK);
        sys.state = STATE_IDLE;
        // Don't run startup script. Prevents stored moves in startup from causing accidents.
      } // Otherwise, no effect.
      break;      
#ifdef KEYME_BOARD
     case 'E': {
       char axis = line[++char_counter];
        if ( axis != 0 ) {
          if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          if ( axis == '1' || axis == '0') {
            counters_enable(axis-'0');
          }
          else {
            axis = get_axis_idx(axis); 
            if (axis == N_AXIS) { return(STATUS_INVALID_STATEMENT); }
            counters_reset(axis);
          }
        }
        return STATUS_ALT_REPORT(REQUEST_COUNTER_REPORT);
     }
      break;
    case 'S':
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      return STATUS_ALT_REPORT(REQUEST_VOLTAGE_REPORT);
      break;
    case 'R':
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      IO_RESET_PORT |= IO_RESET_MASK;  //reset IO.  Will re-enable in loop
      break;
  case 'Z': //zero the current buffers
      if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      SYS_EXEC |= EXEC_FEED_HOLD;  //Stop motion;
      while (sys.state & (STATE_CYCLE)) protocol_execute_runtime(); //spin untill stopped
      st_reset  (); // Immediately force kill steppers and reset step segment buffer.
      plan_reset(); // Reset planner buffer. Zero planner positions. Ensure probe motion is cleared.
      plan_sync_position(); // Sync planner position to current machine position for pull-off move.
      sys.state = STATE_IDLE;
      break;

#endif
         
//  case 'J' : break;  // Jogging methods
    // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be 
    // susceptible to other runtime commands except for e-stop. The jogging function is intended to
    // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped 
    // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
    // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
    // motion by repeatedly toggling to slow the motion to the desired location. Location data would 
    // need to be updated real-time and supplied to the user through status queries.
    //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are 
    // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
    // block buffer without having the planner plan them. It would need to manage de/ac-celerations 
    // on its own carefully. This approach could be effective and possibly size/memory efficient.        
    default : 
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[char_counter] ) {
        case '$' : // Prints Grbl settings [IDLE/ALARM]
          if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_grbl_settings(); }
          break;
        case '#' : // Print Grbl NGC parameters
          if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;

        case 'H' : // Perform homing cycle [IDLE/ALARM], only if idle or lost
          if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
            char axis = line[++char_counter];
            if (axis == '\0' ) {
              axis = HOMING_CYCLE_ALL; //do all axes if none specified
            }
            else {
              if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
              axis = get_axis_idx(axis);
              if (axis == N_AXIS) { return(STATUS_INVALID_STATEMENT); }
              axis = (1<<axis);  //convert idx to mask
            }
            report_status_message(STATUS_OK); //report that we are homing
            mc_homing_cycle(axis);
            if (!sys.abort) { system_execute_startup(line); } // Execute startup scripts after successful homing.
            return STATUS_QUIET_OK; //already said ok
          } else { return(STATUS_SETTING_DISABLED); }
          break;
        case 'I' : // Print or store build info. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) {
            if (!(settings_read_build_info(line))) {
              report_status_message(STATUS_SETTING_READ_FAIL);
            } else {
              report_build_info(line);
              return STATUS_QUIET_OK;
            }
          } else { // Store startup line [IDLE/ALARM]
            if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
            helper_var = char_counter; // Set helper variable as counter to start of user info line.
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            settings_store_build_info(line);
          }
          break;
        case 'N' : // Startup lines. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // Print startup lines
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
            helper_var = true;  // Set helper_var to flag storing method.
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line); // Set helper_var to returned status code.
            if (helper_var) { return(helper_var); }
            else {
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if(line[char_counter] != 0) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting(parameter, value));
          }
      }
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}


// * line num stuff *
#define STLT_SIZE BLOCK_BUFFER_SIZE+1

typedef struct {
  linenumber_t lines[STLT_SIZE];
  uint8_t head;
  uint8_t tail;
} st_linetrack_t;
static st_linetrack_t st_lt;


void linenumber_init() {
  //init line numbering
  st_lt.head = 1;
  st_lt.tail = 0;
  memset(st_lt.lines,BLOCK_BUFFER_SIZE,sizeof(st_lt.lines));
}


uint8_t linenumber_insert(linenumber_t line_number)
{
  if (st_lt.head!=st_lt.tail){
    st_lt.lines[st_lt.head] = line_number;
    if (++st_lt.head>=STLT_SIZE) { st_lt.head = 0;}
  }
  //calculate and return number of items in queue.
  uint8_t head = st_lt.head;
  if (head<=st_lt.tail){ head+=STLT_SIZE;}
  return head-st_lt.tail-1;
}

uint8_t linenumber_next(){
  uint8_t read_idx = st_lt.tail;
  if (++read_idx>=STLT_SIZE) { read_idx=0; }
  return read_idx;
}
uint8_t ln_head() { return st_lt.head;}


linenumber_t linenumber_get(){
  uint8_t read_idx = linenumber_next();
  if (read_idx != st_lt.head) {
    st_lt.tail = read_idx;
    return st_lt.lines[read_idx];
  }
  return 0;
}

linenumber_t linenumber_peek(){
  uint8_t read_idx = linenumber_next();
  if (read_idx != st_lt.head) {
    return st_lt.lines[read_idx];
  }
  return 0;
}

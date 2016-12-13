/*
  gcode.c - rs274/ngc parser.
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
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "probe.h"
#include "report.h"

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1 
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required

#define N_MODAL_GROUPS 8
// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);

float single_step_speed;

void gc_init()
{
  memset(&gc_state, 0, sizeof(gc_state));

  // Load default G54 coordinate system.
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { 
    report_status_message(STATUS_SETTING_READ_FAIL); 
  } 
  { //support for UNITS_MODE_STEP
    uint8_t i;
    single_step_speed = DEFAULT_SINGLE_STEP_RATE / settings.steps_per_mm[0];
    for (i=1; i<N_AXIS; i++) {
      single_step_speed = min(single_step_speed, DEFAULT_SINGLE_STEP_RATE / settings.steps_per_mm[i]);
    }
  }
}

// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position() 
{
  uint8_t i;
  for (i=0; i<N_AXIS; i++) {
    gc_state.position[i] = sys.position[i]/settings.steps_per_mm[i];
  }
}

static uint8_t gc_check_same_position(float *pos_a, float *pos_b) 
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    if (pos_a[idx] != pos_b[idx]) { return(false); }
  }
  return(true);
}

bool gc_is_value_in_array(uint8_t val, uint8_t arr[], uint8_t len)
{
  uint8_t idx;
  for (idx = 0; idx < len; idx++) {
    if (arr[idx] == val)
      return true;
  }
  return false;

}

uint8_t gc_process_modal_group_G0(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)axis_command;
  switch(int_value) {
    case 4: 
      gc_block.non_modal_command = NON_MODAL_DWELL;
      break; // G4
    case 10: 
      gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_DATA;
      break; // G10
    case 28:
      switch(mantissa) {
        case 0: 
          gc_block.non_modal_command = NON_MODAL_GO_HOME_0; 
          break;  // G28
        case 10:
          gc_block.non_modal_command = NON_MODAL_SET_HOME_0;
          break; // G28.1
        default:
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G28.x command]
      }
      mantissa = 0; // Set to zero to indicate valid non-integer G command.
      break;
    case 30: 
      switch(mantissa) {
        case 0:
          gc_block.non_modal_command = NON_MODAL_GO_HOME_1;  // G30
          break;
        case 10:
          gc_block.non_modal_command = NON_MODAL_SET_HOME_1;  // G30.1
          break;
        default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G30.x command]
      }
      mantissa = 0; // Set to zero to indicate valid non-integer G command.
      break;
    case 53: 
      gc_block.non_modal_command = NON_MODAL_ABSOLUTE_OVERRIDE; 
      break; // G53
    case 92: 
      switch(mantissa) {
        case 0: 
          gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_OFFSET;
          break; // G92
        case 10: 
          gc_block.non_modal_command = NON_MODAL_RESET_COORDINATE_OFFSET;
          break; // G92.1
        default:
          // [Unsupported G92.x command]
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);      }
      mantissa = 0; // Set to zero to indicate valid non-integer G command.
      break;      
  }
  return STATUS_GCODE_NO_FAIL; 
}

uint8_t gc_process_modal_group_G1(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  switch(int_value) {
    case 0: case 1: case 2: case 3: case 38: 
    // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
    // * G43.1 is also an axis command but is not explicitly defined this way.
      if (*axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
      *axis_command = AXIS_COMMAND_MOTION_MODE; 
      // No break. Continues to next line.
    case 80: 
      switch(int_value) {
        case 0:
          gc_block.modal.motion = MOTION_MODE_SEEK;  // G0
          break;
        case 1:
          gc_block.modal.motion = MOTION_MODE_LINEAR; // G1
          break;
        case 2:
          gc_block.modal.motion = MOTION_MODE_CW_ARC;  // G2
          break;
        case 3:
          gc_block.modal.motion = MOTION_MODE_CCW_ARC;  // G3
          break;
        case 38: 
          switch(mantissa) {
            case 20:  // G38.2
              gc_block.modal.motion = MOTION_MODE_PROBE; 
              break;
            /* NOTE: If G38.3+ are enabled, change mantissa variable type to uint16_t.
               G38.3 - Not supported.
               G38.4 - Not supported.
               G38.5 - Not supported. */
            default:
              // Unsupported G38.x command
              FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
          }
          mantissa = 0; // Set to zero to indicate valid non-integer G command.
          break;
        case 80: gc_block.modal.motion = MOTION_MODE_NONE; break; // G80
      }            
      break;
  }
  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_modal_group_G2(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)mantissa;
  (void)axis_command;

  switch(int_value) {
    case 17:
      gc_block.modal.plane_select = PLANE_SELECT_XY;
      break;
    case 18:
      gc_block.modal.plane_select = PLANE_SELECT_ZX;
      break;
    case 19:
      gc_block.modal.plane_select = PLANE_SELECT_YZ;
      break;
  }
  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_modal_group_G3(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)mantissa;
  (void)axis_command;

  if (int_value == 90) {
    gc_block.modal.distance = DISTANCE_MODE_ABSOLUTE;  // G90
  } else {
    gc_block.modal.distance = DISTANCE_MODE_INCREMENTAL;  // G91
  }

  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_modal_group_G5(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)mantissa;
  (void)axis_command;
  if (int_value == 93)
    gc_block.modal.feed_rate = FEED_RATE_MODE_INVERSE_TIME;  // G93
  else
    gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;  // G94

  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_modal_group_G6(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)mantissa;
  (void)axis_command;
  switch(int_value) {
    case 20: case 21: 
      if (int_value == 20)
        gc_block.modal.units = UNITS_MODE_INCHES;  // G20
      else
        gc_block.modal.units = UNITS_MODE_MM;  // G21
      break;
    case 66:  //KEYME units steps extension
      gc_block.modal.units = UNITS_MODE_STEP;  // G21
      break;
  }

  return STATUS_GCODE_NO_FAIL;

}

uint8_t gc_process_modal_group_G8(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  // NOTE: The NIST g-code standard vaguely states that when a tool length
  // offset is changed, there cannot be any axis motion or coordinate offsets
  // updated. Meaning G43, G43.1, and G49 all are explicit axis commands,
  // regardless if they require axis words or not.

  if (*axis_command) {
    // [Axis word/command conflict] }
    FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); 
  }  
 
  *axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
  
  if (int_value == 49) { // G49
    gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL; 
  } else if (mantissa == 10) { // G43.1
    gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
  } else { 
    FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G43.x command]
  }  

  // Set to zero to indicate valid non-integer G command. 
  mantissa = 0; 

  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_modal_group_G12(uint8_t int_value, uint8_t mantissa, uint8_t * axis_command)
{
  (void)mantissa;
  (void)axis_command;

  // NOTE: G59.x are not supported. (But their
  // int_values would be 60, 61, and 62.)
  gc_block.modal.coord_select = int_value - 54; // Shift to array indexing.
  
  return STATUS_GCODE_NO_FAIL;
}

static const struct g_modal {
  uint8_t word_bit;
  // Min and max are used to make searching through
  // arrays more efficient.
  uint8_t list[10];
  uint8_t min;
  uint8_t max;
  uint8_t (*fun)(uint8_t, uint8_t, uint8_t *);
  uint8_t len;

} g_modal_list[N_MODAL_GROUPS] = {
  {
    .word_bit = MODAL_GROUP_G0,
    .list = {4, 10, 28, 30, 53, 92},
    .min = 4,
    .max = 92,
    .fun = gc_process_modal_group_G0,
    .len = 6
  },
  {
    .word_bit = MODAL_GROUP_G1,
    .list = {0, 1, 2, 3, 38, 80},
    .min = 0,
    .max = 80,
    .fun = gc_process_modal_group_G1,
    .len = 6
  },
  {
    .word_bit = MODAL_GROUP_G2,
    .list = {17, 18, 19},
    .min = 17,
    .max = 19,
    .fun = gc_process_modal_group_G2,
    .len = 3
  },
  {
    .word_bit = MODAL_GROUP_G3,
    .list = {90, 91},
    .min = 90,
    .max = 91,
    .fun = gc_process_modal_group_G3,
    .len = 2
  },
  {
    .word_bit = MODAL_GROUP_G5,
    .list = {93, 94},
    .min = 93,
    .max = 94,
    .fun = gc_process_modal_group_G5,
    .len = 2
  },
  {
    .word_bit = MODAL_GROUP_G6,
    .list = {20, 21},
    .min = 20,
    .max = 21,
    .fun = gc_process_modal_group_G6,
    .len = 2
  },
  {
    .word_bit = MODAL_GROUP_G8,
    .list = {43, 49},
    .min = 43,
    .max = 49,
    .fun = gc_process_modal_group_G8,
    .len = 2
  },
  {
    .word_bit = MODAL_GROUP_G12,
    .list = {54, 55, 56, 57, 58, 59},
    .min = 54,
    .max = 59,
    .fun = gc_process_modal_group_G12,
    .len = 6
  }

};

uint8_t gc_get_g_modal_group(int int_value)
{
  // TODO Change to binary search for a VERY minor improvement
  for(uint8_t i = 0; i < N_MODAL_GROUPS; i++) {
    if (int_value <= g_modal_list[i].max &&
        int_value >= g_modal_list[i].min) {
        for(uint8_t j = 0; j < g_modal_list[i].len; j++) {
          if (g_modal_list[i].list[j] == int_value)
            return i;
        }
    }
  }
  return N_MODAL_GROUPS;
}


uint8_t gc_process_m_commands(int int_value, int mantissa,
                              uint16_t * command_words, uint8_t * word_bit)
{
  // Determine 'M' command and its modal group
  if (mantissa > 0)
    FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); // [No Mxx.x commands]
  
  switch(int_value) {
    case 0: case 1: case 2: case 30: 
      *word_bit = MODAL_GROUP_M4; 
      switch(int_value) {
      case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
      case 1: break; // Optional stop not supported. Ignore.
      case 2: case 30: gc_block.modal.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset 
    }
      break;
    case 3: case 4: case 5: 
      *word_bit = MODAL_GROUP_M7; 
      switch(int_value) {
        case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
        case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
        case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
      }
      break;            
    #ifdef ENABLE_M7  
      case 7:
    #endif
    case 8: case 9:
      *word_bit = MODAL_GROUP_M8; 
      switch(int_value) {      
        #ifdef ENABLE_M7
          case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
        #endif
        case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
        case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
      }
      break;
    default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
  }

  // Check for more than one command per modal group violations in the current block
  // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
  if (bit_istrue(*command_words,bit(*word_bit))) {
    FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION);
  }
  
  bit_true(*command_words,bit(*word_bit));

  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_process_other_commands(char letter, uint8_t * word_bit,
  uint8_t * axis_words, uint8_t * ijk_words, float value, uint8_t int_value, uint16_t * value_words)
{
  switch(letter) {
    // case 'A': // Not supported
    // case 'B': // Not supported
    case 'C':
      *word_bit = WORD_C;
      gc_block.values.xyz[C_AXIS] = value;
      *axis_words |= (1<<C_AXIS);
      break;
    // case 'D': // Not supported
    case 'F':
      *word_bit = WORD_F;
      gc_block.values.f = value;
      break;
    // case 'H': // Not supported
    case 'I':
      *word_bit = WORD_I;
      gc_block.values.ijk[X_AXIS] = value;
      *ijk_words |= (1<<X_AXIS);
      break;
    case 'J':
      *word_bit = WORD_J;
      gc_block.values.ijk[Y_AXIS] = value;
      *ijk_words |= (1<<Y_AXIS);
      break;
    case 'K':
      *word_bit = WORD_K;
      gc_block.values.ijk[Z_AXIS] = value;
      *ijk_words |= (1<<Z_AXIS);
      break;
    case 'L':
      *word_bit = WORD_L;
      gc_block.values.l = int_value;
      break;
    case 'N':
      *word_bit = WORD_N;
      gc_block.values.n = trunc(value);
      break;
    case 'P':
      *word_bit = WORD_P;
      gc_block.values.p = value;
      break;
    // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
    // case 'Q': // Not supported
    case 'R':
      *word_bit = WORD_R;
      gc_block.values.r = value;
      break;
    case 'S':
      *word_bit = WORD_S;
      gc_block.values.s = value;
      break;
    case 'T':
      *word_bit = WORD_T;
      break; // gc.values.t = int_value;
    case 'X':
      *word_bit = WORD_X;
      gc_block.values.xyz[X_AXIS] = value;
      *axis_words |= (1<<X_AXIS);
      break;
    case 'Y':
      *word_bit = WORD_Y;
      gc_block.values.xyz[Y_AXIS] = value;
      *axis_words |= (1<<Y_AXIS);
      break;
    case 'Z':
      *word_bit = WORD_Z;
      gc_block.values.xyz[Z_AXIS] = value;
      *axis_words |= (1<<Z_AXIS);
      break;
    default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
  } 

  // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
  if (bit_istrue(*value_words, bit(*word_bit))) {
    FAIL(STATUS_GCODE_WORD_REPEATED);  // [Word repeated]
  }

  // Check for invalid negative values for words F, N, P, T, and S.
  // NOTE: Negative value check is done here simply for code-efficiency.
  const uint16_t check_bits = (bit(WORD_F) | bit(WORD_N) | bit(WORD_P) |
                              bit(WORD_T) | bit(WORD_S));
  if (bit(*word_bit) & check_bits) {
    if (value < 0.0) {
      FAIL(STATUS_NEGATIVE_VALUE);  // [Word value cannot be negative]
    }
  }
  *value_words |= bit(*word_bit); // Flag to indicate parameter assigned.

  return STATUS_GCODE_NO_FAIL;
}

uint8_t gc_import_gcode_words(char * line, uint8_t * axis_words, uint16_t * command_words,
                              uint8_t * ijk_words, uint16_t * value_words, uint8_t * axis_command,
                              uint8_t *int_value)
{
  /*---------------------------------------------------------------------------
    STEP 2: Import all g-code words in the block line. A g-code word is a 
    letter followed by a number, which can either be a 'G'/'M' command or
    sets/assigns a command value. Also, perform initial error-checks for
    command word modal group violations, for any repeated words, and for
    negative values set for the value words F, N, P, T, and S. 
  */
     
  uint8_t word_bit; // Bit-value for assigning tracking variables
  uint8_t char_counter = 0;  
  char letter;
  float value;
  // NOTE: For mantissa values > 255, variable type must be changed to uint16_t.
  uint8_t mantissa = 0; 

  while (line[char_counter] != 0) { // Loop until no more g-code words in line. 
    // Import the next g-code word, expecting a letter followed by a value.
    // Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) {
      FAIL(STATUS_EXPECTED_COMMAND_LETTER);  // [Expected word letter]
    }

    char_counter++;
    if (!read_float(line, &char_counter, &value)) {
      FAIL(STATUS_BAD_NUMBER_FORMAT);  // [Expected word value]
    }

    // Convert values to smaller uint8 significand and mantissa values
    // for parsing this word.
    // NOTE: Mantissa is multiplied by 100 to catch non-integer command values.
    // This is more accurate than the NIST gcode requirement of x10 when used 
    // for commands, but not quite accurate enough for value words that require
    // integers to within 0.0001. This should be a good enough comprimise and
    // catch most all non-integer errors. To make it compliant, we would simply
    // need to change the mantissa to int16, but this add compiled flash space.
    // Maybe update this later. 
    *int_value = trunc(value);
    mantissa = round(100 * (value - *int_value)); // Compute mantissa for Gxx.x commands.
      // NOTE: Rounding must be used to catch small floating point errors. 

    if (letter == 'G') {
      uint8_t g_idx;
      g_idx = gc_get_g_modal_group(*int_value);
      
      if (g_idx ==  N_MODAL_GROUPS) {
        FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
      }

      word_bit = g_modal_list[g_idx].word_bit;
      g_modal_list[g_idx].fun(*int_value, mantissa, axis_command);
    } else if (letter == 'M') {
      gc_process_m_commands(*int_value, mantissa, command_words, &word_bit);
    } else {
      gc_process_other_commands(letter, &word_bit, axis_words, ijk_words,
                                value, *int_value, value_words);
    }       

    // Check if the g-code word is supported or errors due to modal group 
    // violations or has been repeated in the g-code block. If ok, update the
    // command or record its value.

    // Non-Command Words: This initial parsing phase only checks for repeats
    // of the remaining legal g-code words and stores their value. 
    // Error-checking is performed later since some words (I,J,K,L,P,R) have
    // multiple connotations and/or depend on the issued commands.

  }
  // Parsing complete!
  return(STATUS_GCODE_NO_FAIL);
}

uint8_t gc_check_errors(uint8_t axis_words, uint8_t * axis_command, uint16_t * value_words,
                        uint8_t * axis_0, uint8_t * axis_1, uint8_t * axis_linear,
                        float * coordinate_data, uint16_t * command_words, uint8_t * int_value,
                        float * parameter_data, uint8_t ijk_words)
{
  /* STEP 3: Error-check all commands and values passed in this function. */
 
  /*
     [0. Non-specific/common error-checks and miscellaneous setup]: 
  
     Determine implicit axis command conditions. Axis words have been passed,
     but no explicit axis command has been sent. If so, set axis command to
     current motion mode.
  */

  if (axis_words) {
    if (!(*axis_command))
      // Assign implicit motion-mode
      *axis_command = AXIS_COMMAND_MOTION_MODE;
  }
  // Check for valid line number N value.
  if (bit_istrue(*value_words, bit(WORD_N))) {
    // Line number value cannot be less than zero (done) or greater
    // than max line number.
    if (gc_block.values.n > LINENUMBER_MAX)
      FAIL(STATUS_GCODE_INVALID_LINE_NUMBER);  // [Exceeds max line number]
  }
  
  /*
     NOTE: Single-meaning value word. Set at end of error-checking.
     bit_false(value_words,bit(WORD_N)); 

     Track for unused words at the end of error-checking.
     NOTE: Single-meaning value words are removed all at once at the end of
     error-checking, because they are always used when present. This was done
     to save a few bytes of flash. For clarity, the single-meaning value words
     may be removed as they are used. Also, axis words are treated in the same
     way. If there is an explicit/implicit axis command, XYZ words are always
     used and are removed at the end of error-checking.  
  */

  /* [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol. */
  
  /*
    [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active,
    implicitly or explicitly. Feed rate is not defined after switching
    to G94 from G93.
  */
  
  if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
    // NOTE: G38 can also operate in inverse time, but is undefined as an error.
    // Missing F word check added here.
    if (*axis_command == AXIS_COMMAND_MOTION_MODE) { 
      if ((gc_block.modal.motion != MOTION_MODE_NONE) 
          || (gc_block.modal.motion != MOTION_MODE_SEEK)) {
        if (bit_isfalse(*value_words, bit(WORD_F)))
          FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE);  // [F word missing]
      }
    }
    if (gc_block.modal.units == UNITS_MODE_STEP)
      FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);  //KEYME TODO ERROR
 
    /*
      NOTE: It seems redundant to check for an F word to be passed after
      switching from G94 to G93. We would accomplish the exact same thing if
      the feed rate value is always reset to zero and undefined after each
      inverse time block, since the commands that use this value already
      perform undefined checks. This would also allow other commands,
      following this switch, to execute and not error out needlessly. This
      code is combined with the above feed rate mode and the below set feed
      rate error-checking.
    */
    
    /*
      [3. Set feed rate ]: F is negative (done.)
       - In inverse time mode: Always implicitly zero the feed rate value
       before and after block completion.
    
       NOTE: If in G93 mode or switched into it from G94, just keep F value
       as initialized zero or passed F word value in the block. If no F word is
       passed with a motion command that requires a feed rate, this will error
       out in the motion modes error-checking. However, if no F word is passed
       with NO motion command that requires a feed rate, we simply move on and
       the state feed rate value gets updated to zero and remains undefined.
    */

  } else { // = G94
    /* - In units per mm mode: If F word passed, ensure value is in mm/min,
       otherwise push last state value. */
    if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { 
      // Last state is also G94
      if (bit_istrue(*value_words,bit(WORD_F))) {
        if (gc_block.modal.units == UNITS_MODE_INCHES) 
          gc_block.values.f *= MM_PER_INCH;
        else if (gc_block.modal.units == UNITS_MODE_STEP)
          gc_block.values.f = single_step_speed; //TODO: not sure about this yet.
      } else {
        gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
      }
    } // Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
  } 
  // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking.
  
  // [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(*value_words, bit(WORD_S)))
    gc_block.values.s = gc_state.spindle_speed;
  // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking. TODO: Why is this commented out?
    
  /* [5-9]
     [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
     // bit_false(value_words,bit(WORD_T)); // NOTE: Single-meaning value word. Set at end of error-checking. TODO: WHy is this commented out

     [6. Change tool ]: N/A
     [7. Spindle control ]: N/A
     [8. Coolant control ]: N/A
     [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED.
  */

  /* [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below. */
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(*value_words, bit(WORD_P)))
      FAIL(STATUS_GCODE_VALUE_WORD_MISSING);  // [P word missing]
    bit_false(*value_words, bit(WORD_P));
  }
  
  /* [11. Set active plane ]: N/A */
  switch (gc_block.modal.plane_select) {  //TODO: keyme only supports XY?
    case PLANE_SELECT_XY:
      *axis_0 = X_AXIS;
      *axis_1 = Y_AXIS;
      *axis_linear = Z_AXIS;
      break;
    case PLANE_SELECT_ZX:
      *axis_0 = Z_AXIS;
      *axis_1 = X_AXIS;
      *axis_linear = Y_AXIS;
      break;
    default: // case PLANE_SELECT_YZ:
      *axis_0 = Y_AXIS;
      *axis_1 = Z_AXIS;
      *axis_linear = X_AXIS;
  }
            
  /* [12. Set length units ]: N/A */
  /* Pre-convert XYZ coordinate values to millimeters, if applicable. */
  uint8_t idx;
  if (gc_block.modal.units != UNITS_MODE_MM) {
    for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
      if (bit_istrue(axis_words, bit(idx)) ) {
        if (gc_block.modal.units == UNITS_MODE_STEP ){ //Keyme units step extension
          gc_block.values.xyz[idx] /= settings.steps_per_mm[idx];  
        }
        else {
          gc_block.values.xyz[idx] *= MM_PER_INCH;
        }
      }
    }
  }
  
  /* [13. Cutter radius compensation ]: NOT SUPPORTED. Error, if G53 is active. */

  /*
    [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are. 
    [G43.1 Errors]: Motion command in same line. 
      NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid 
      axis that is configured (in config.h). There should be an error if the configured axis
      is absent or if any of the other axis words are present.
  */
  if (*axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates called in block.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
      if (axis_words ^ (1 << TOOL_LENGTH_OFFSET_AXIS))
        FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR);
    }
  }
  
  /*
    [15. Coordinate system selection ]: *N/A. Error, if cutter radius comp is active.
    TODO: An EEPROM read of the coordinate data may require a buffer sync when the cycle
    is active. The read pauses the processor temporarily and may cause a rare crash. For 
    future versions on processors with enough memory, all coordinate data should be stored
    in memory and written to EEPROM only when there is not a cycle active.
  */
  memcpy(coordinate_data, gc_state.coord_system, sizeof(gc_state.coord_system));
  if (bit_istrue(*command_words, bit(MODAL_GROUP_G12))) { // Check if called in block
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) {
      FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS);  // [Greater than N sys]
    }
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select, coordinate_data))) {
        FAIL(STATUS_SETTING_READ_FAIL); } 
    }
  }
  
  /* [16]-18]
    [16. Set path control mode ]: NOT SUPPORTED.
    [17. Set distance mode ]: N/A. G90.1 and G91.1 NOT SUPPORTED.
    [18. Set retract mode ]: NOT SUPPORTED.
  */
  
  /*
    [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets. 
    NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
    commands all treat axis words differently. G10 as absolute offsets or computes current position as
    the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
    all the current coordinate system and G92 offsets. 
  */
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:  
      /* [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
         [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to nCoordSys(max 9). Axis words missing.
         [G10 L20 Errors]: P must be 0 to nCoordSys(max 9). Axis words missing. */
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS) }; // [No axis words]
      if (bit_isfalse(*value_words, ((1 << WORD_P) | (1 << WORD_L))))
        FAIL(STATUS_GCODE_VALUE_WORD_MISSING);  // [P/L word missing]
      *int_value = trunc(gc_block.values.p); // Convert p value to int.
      if (*int_value > N_COORDINATE_SYSTEM) {
        FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); // [Greater than N sys]
      }
      if (gc_block.values.l != 20) {
        if (gc_block.values.l == 2) {
          if (bit_istrue(*value_words, bit(WORD_R))) {
            FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);  // [G10 L2 R not supported]
          }
        } else {
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);  // [Unsupported L]
        }
      }
      bit_false(*value_words, (bit(WORD_L) | bit(WORD_P)));
    
      // Load EEPROM coordinate data and pre-calculate the new coordinate data.
      if (*int_value > 0) { 
        (*int_value)--;  // Adjust P1-P6 index to EEPROM coordinate data indexing.
      } else {
        *int_value = gc_block.modal.coord_select;  // Index P0 as the active coordinate system
      }      

      if (!settings_read_coord_data(*int_value, parameter_data)) {
        FAIL(STATUS_SETTING_READ_FAIL);  // [EEPROM read fail]
      }

      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        if (bit_istrue(axis_words, bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20: Update coordinate system axis at current position (with modifiers) with programmed value
            parameter_data[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
            if (idx == TOOL_LENGTH_OFFSET_AXIS) { parameter_data[idx] -= gc_state.tool_length_offset; }
          } else {
            // L2: Update coordinate system axis to programmed value.
            parameter_data[idx] = gc_block.values.xyz[idx]; 
          }
        }
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92 Errors]: No axis words.
      if (!axis_words) {
        FAIL(STATUS_GCODE_NO_AXIS_WORDS);  // [No axis words]
      }
    
      // Update axes defined only in block. Offsets current system to defined value. Does not update when
      // active coordinate system is selected, but is still active unless G92.1 disables it. 
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        if (bit_istrue(axis_words,bit(idx)) ) {
          gc_block.values.xyz[idx] = gc_state.position[idx]-coordinate_data[idx]-gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] -= gc_state.tool_length_offset; }
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;
      
    default:

      /* At this point, the rest of the explicit axis commands treat the axis values as the traditional
         target position with the coordinate system offsets, G92 offsets, absolute override, and distance
         modes applied. This includes the motion mode commands. We can now pre-compute the target position. */
      // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
      if (*axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO block any axis command.
        if (axis_words) {
          for (idx=0; idx < N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
            if ( bit_isfalse(axis_words,bit(idx)) ) {
              // No axis word in block. Keep same axis position.
              gc_block.values.xyz[idx] = gc_state.position[idx];
            } else {
              // Update specified value according to distance mode or ignore if absolute override is active.
              // NOTE: G53 is never active with G28/30 since they are in the same modal group.
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // Apply coordinate offsets based on distance mode.
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                  gc_block.values.xyz[idx] += coordinate_data[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // Incremental mode
                  gc_block.values.xyz[idx] += gc_state.position[idx];
                }
              }
            }
          }
        }
      }
          
      // Check remaining non-modal commands for errors.
      switch (gc_block.non_modal_command) {
        case NON_MODAL_GO_HOME_0: 
          // [G28 Errors]: Cutter compensation is enabled. 
          // Retreive G28 go-home position data (in machine coordinates) from EEPROM
          if (!settings_read_coord_data(SETTING_INDEX_G28,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_GO_HOME_1:
          // [G30 Errors]: Cutter compensation is enabled. 
          // Retreive G30 go-home position data (in machine coordinates) from EEPROM
          if (!settings_read_coord_data(SETTING_INDEX_G30,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_SET_HOME_0: case NON_MODAL_SET_HOME_1:
          // [G28.1/30.1 Errors]: Cutter compensation is enabled. 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET: 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
          // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 not active]
          }
          break;
      }
  }
      
  // [20. Motion modes ]: 
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
    // [G80 Errors]: Axis word exist and are not used by a non-modal command.
    if ((axis_words) && (*axis_command != AXIS_COMMAND_NON_MODAL)) { 
      FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); // [No axis words allowed]
    }

  // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or 
  // was explicitly commanded in the g-code block.
  } else if (*axis_command == AXIS_COMMAND_MOTION_MODE) {
  
    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
      // [G0 Errors]: Axis letter not configured or without real value (done.)
      // Axis words are optional. If missing, set axis command flag to ignore execution.
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

    // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
    // the value must be positive. In inverse time mode, a positive value must be passed with each block.
    } else {      
      // Check if feed rate is defined for the motion modes that require it.
      if (gc_block.values.f == 0.0) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [Feed rate undefined]
     
      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR: 
          // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
          // Axis words are optional. If missing, set axis command flag to ignore execution.
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

          break;
        case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
          // [G2/3 Errors All-Modes]: Feed rate undefined.
          // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
          // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current 
          //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).   
          // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.        
          // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.
        
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (!(axis_words & (bit(*axis_0) | bit(*axis_1)))) {
            FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE);  // [No axis words in plane]
          } else if (gc_block.modal.units == UNITS_MODE_STEP) {
            FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION);  //can't do arc by steps.
                                                       //TODO: what's the best errror code? 
          }
          // Calculate the change in position along each selected axis    
          // Delta x between current position and target
          float x = gc_block.values.xyz[*axis_0] - gc_state.position[*axis_0];

          // Delta y between current position and target
          float y = gc_block.values.xyz[*axis_1] - gc_state.position[*axis_1]; 

          if (*value_words & bit(WORD_R)) { // Arc Radius Mode  
            bit_false(*value_words, bit(WORD_R));
            if (gc_check_same_position(gc_state.position, gc_block.values.xyz))
              FAIL(STATUS_GCODE_INVALID_TARGET);  // [Invalid target]
          
            // Convert radius value to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES)
              gc_block.values.r *= MM_PER_INCH;
        
            /* First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
            than d. If so, the sqrt of a negative number is complex and error out. */
            float h_x2_div_d = 4.0 * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { 
              FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); // [Arc radius error]
            }

            // Finish computing h_x2_div_d.
            h_x2_div_d = -sqrt(h_x2_div_d) / hypot_f(x, y); // == -(h * 2 / d)
            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) {
              h_x2_div_d = -h_x2_div_d;
            }

            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
            // even though it is advised against ever generating such circles in a single line of g-code. By 
            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
            // travel and thus we get the unadvisably long arcs as prescribed.
            if (gc_block.values.r < 0) { 
                h_x2_div_d = -h_x2_div_d; 
                gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
            }        
            // Complete the operation by calculating the actual center of the arc
            gc_block.values.ijk[*axis_0] = 0.5 * (x - (y * h_x2_div_d));
            gc_block.values.ijk[*axis_1] = 0.5 * (y + (x * h_x2_div_d));
          
          } else { // Arc Center Format Offset Mode  
            if (!(ijk_words & (bit(*axis_0) | bit(*axis_1))))
              FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE);  // [No offsets in plane]
            bit_false(*value_words,(bit(WORD_I) | bit(WORD_J) | bit(WORD_K)));  
          
            // Convert IJK values to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx < N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
                if (ijk_words & bit(idx))
                  gc_block.values.ijk[idx] *= MM_PER_INCH;
              }
            }         

            // Arc radius from center to target
            x -= gc_block.values.ijk[*axis_0]; // Delta x between circle center and target
            y -= gc_block.values.ijk[*axis_1]; // Delta y between circle center and target
            float target_r = hypot_f(x,y); 

            // Compute arc radius for mc_arc. Defined from current location to center.
            gc_block.values.r = hypot_f(gc_block.values.ijk[*axis_0], gc_block.values.ijk[*axis_1]); 
            
            // Compute difference between current location and target radii for final error-checks.
            float delta_r = fabs(target_r-gc_block.values.r);
            if (delta_r > 0.005) { 
              if (delta_r > 0.5) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.5mm
              if (delta_r > (0.001*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.005mm AND 0.1% radius
            }
          }
          break;
        case MOTION_MODE_PROBE:
          if (bit_istrue(*value_words, bit(WORD_P)))
            bit_false(*value_words, bit(WORD_P));
          else
            FAIL(STATUS_GCODE_NO_PROBE_SENSOR_SPECIFIED);
          // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
          //   is undefined. Probe is triggered.
          //KeyMe: same posiiton will report probe fail when done.
          //KeyMe: Already triggered will report success
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          //XX if (gc_check_same_position(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          //XX if (probe_get_state()) { FAIL(STATUS_GCODE_PROBE_TRIGGERED); } // [Probe triggered]
          break;
      } 
    }
  }
  
  // [21. Program flow ]: No error check required.

  // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
  // radius mode, or axis words that aren't used in the block.  
  bit_false(*value_words, (bit(WORD_N) | bit(WORD_F) | bit(WORD_S) | bit(WORD_T)));  // Remove single-meaning value words. 
  if (axis_command)
    bit_false(*value_words, (bit(WORD_X) | bit(WORD_Y) | bit(WORD_Z) | bit(WORD_C)));  // Remove axis words. 
  if (*value_words)
    FAIL(STATUS_GCODE_UNUSED_WORDS);  // [Unused words]
  
  return(STATUS_GCODE_NO_FAIL);

}

uint8_t gc_execute_block(const uint8_t axis_command, float * coordinate_data,
                         float * parameter_data, const uint8_t axis_0,
                         const uint8_t axis_1, const uint8_t axis_linear,
                         uint8_t * retval) 
{
  /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
  */ 
    
  uint8_t int_value;

  /* [1. Comments feedback ]:  NOT SUPPORTED */
  
  /* [2. Set feed rate mode ]: */
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  
  /* [3. Set feed rate ]: */
  gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.

  /* [4. Set spindle speed ]: */
  if (gc_state.spindle_speed != gc_block.values.s) { 
    gc_state.spindle_speed = gc_block.values.s; 
    
    /* Update running spindle only if not in check mode and not already enabled. */
    if (gc_state.modal.spindle != SPINDLE_DISABLE)
      spindle_run(gc_state.modal.spindle, gc_state.spindle_speed);
  }
    
  /* [5. Select tool ]: NOT SUPPORTED
     [6. Change tool ]: NOT SUPPORTED */

  /* [7. Spindle control ]: */
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
    gc_state.modal.spindle = gc_block.modal.spindle;    
    /* Update spindle control and apply spindle speed
       when enabling it in this block. */
    spindle_run(gc_state.modal.spindle, gc_state.spindle_speed);
  }

  /* [8. Coolant control ]: */
  if (gc_state.modal.coolant != gc_block.modal.coolant) {
    gc_state.modal.coolant = gc_block.modal.coolant;
    coolant_run(gc_state.modal.coolant);
  }
  
  /* [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED */

  /* [10. Dwell ]: */
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { 
    linenumber_insert(gc_block.values.n); 
    mc_dwell(gc_block.values.p); 
    request_eol_report(); //pop the number we just put in 
  }
  
  /* [11. Set active plane ]: */
  gc_state.modal.plane_select = gc_block.modal.plane_select;  

  // [12. Set length units ]:
  gc_state.modal.units = gc_block.modal.units;

  /* [13. Cutter radius compensation ]: NOT SUPPORTED */

  /* [14. Cutter length compensation ]: G43.1 and G49 supported. G43 NOT SUPPORTED.
     NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
     of execution. The error-checking step would simply load the offset value into the correct
     axis of the block XYZ value array. */
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates a change.
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) { // G43.1
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
    } else { // G49
      gc_state.tool_length_offset = 0.0;
    }
  }
  
  /* [15. Coordinate system selection ]: */
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,coordinate_data,sizeof(*coordinate_data));
  }
  
  /* [16. Set path control mode ]: NOT SUPPORTED */
  
  /* [17. Set distance mode ]: */
  gc_state.modal.distance = gc_block.modal.distance;
  
  /* [18. Set retract mode ]: NOT SUPPORTED */
    
  /* [19. Go to predefined position, Set G10, or Set axis offsets ]: */
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      // TODO: See if I can clean up this int_value.
      int_value = trunc(gc_block.values.p);  // Convert p value to int.
      if (int_value > 0)
        int_value--;  // Adjust P1-P6 index to EEPROM coordinate data indexing.
      else
        int_value = gc_state.modal.coord_select;  // Index P0 as the active coordinate system
      
      settings_write_coord_data(int_value, parameter_data);
      // Update system coordinate system if currently active.
      if (gc_state.modal.coord_select == int_value) 
        memcpy(gc_state.coord_system, parameter_data, sizeof(*parameter_data));
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1: 
   /* Move to intermediate position before going home. Obeys current coordinate system and offsets 
      and absolute and incremental modes. */
      if (axis_command) {
        mc_line(gc_block.values.xyz, -1.0, false, gc_block.values.n);
      }
      mc_line(parameter_data, -1.0, false, gc_block.values.n); 
      memcpy(gc_state.position, parameter_data, sizeof(*parameter_data));
      break;
    case NON_MODAL_SET_HOME_0: 
      settings_write_coord_data(SETTING_INDEX_G28, gc_state.position);
      break;
    case NON_MODAL_SET_HOME_1:
      settings_write_coord_data(SETTING_INDEX_G30, gc_state.position);
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET: 
      clear_vector(gc_state.coord_offset); // Disable G92 offsets by zeroing offset vector.
      break;
  }

/* [20. Motion modes ]:
   NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes. 
   Enter motion modes only if there are axis words or a motion mode command word in the block. */
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      switch (gc_state.modal.motion) {
        case MOTION_MODE_SEEK:
          mc_line(gc_block.values.xyz, -1.0, false, gc_block.values.n);
          break;
        case MOTION_MODE_LINEAR:
          mc_line(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, gc_block.values.n);
          break;
        case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
          mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
            gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, gc_block.values.n);  
          break;
        case MOTION_MODE_PROBE: {
          probe_move_to_sensor(gc_block.values.xyz, gc_state.feed_rate,
          gc_state.modal.feed_rate, gc_block.values.n, gc_block.values.p); 
          *retval = STATUS_QUIET_OK;
          //block.values.xyz is updated inside probe_cycle, so the next line is correct
        }
      }
    
   /* As far as the parser is concerned, the position is now == target. In reality the
      motion control system might still be processing the action and the real tool position
      in any intermediate location. */
      memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc.position[] = target[];
    }
  }
  
  /* [21. Program flow ]:
     M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may 
     refill and can only be resumed by the cycle start run-time command. */
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) { 
    protocol_buffer_synchronize(); // Finish all remaining buffered motions. Program paused when complete.
    sys.flags &= ~SYSFLAG_AUTOSTART; // Disable auto cycle start. Forces pause until cycle start issued.
  
    /* If complete, reset to reload defaults (G92.2,G54,G17,G90,G94,M48,G40,M5,M9). Otherwise,
       re-enable program flow after pause complete, where cycle start will resume the program. */
    if (gc_state.modal.program_flow == PROGRAM_FLOW_COMPLETED) { mc_reset(); }
    else { gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; }
  }

  return(STATUS_GCODE_NO_FAIL);
}

/*
  Executes one line of 0-terminated G-Code. The line is assumed to contain
  only uppercase characters and signed floating point values (no whitespace).
  Comments and block delete characters have been removed. In this function,
  all units and positions are converted and exported to grbl's internal
  functions in terms of (mm, mm/min) and absolute machine coordinates,
  respectively.
*/
 
uint8_t gc_execute_line(char *line) 
{
  /* ---------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state
     modes. The parser updates these modes and commands as the block line is
     parser and will only be used and executed after successful error-checking.
     The parser block struct also contains a block values struct, word tracking
     variables, and a non-modal commands tracker for the new block. This struct
     contains all of the necessary information to execute the block. 
  */
  
  memset(&gc_block, 0, sizeof(gc_block)); // Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); // Copy current modes
  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  
  /* Multi-use variable to store coordinate data for execution */
  float coordinate_data[N_AXIS]; 
  /* Multi-use variable to store parameter data for execution */
  float parameter_data[N_AXIS];  

  /* Initialize bitflag tracking variables for axis indices compatible operations. */
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking 

  /* Initialize command and value words variables.
     Tracks words contained in this block. */
  
  /* G and M command words. Also used for modal group violations. */
  uint16_t command_words = 0;
  /* Value words */
  uint16_t value_words = 0;

  uint8_t retval = STATUS_OK;

  uint8_t fail_status = -1;  // Used to keep track of step failures
  /* STEP2: Import all g-code words in the block line. */
  uint8_t int_value = 0;
  fail_status = gc_import_gcode_words(line, &axis_words, &command_words, &ijk_words,
                        &value_words, &axis_command, &int_value);
  if (fail_status != STATUS_GCODE_NO_FAIL)
    return fail_status;

  /* STEP 3: Error-check all commandsi */
  gc_check_errors(axis_words, &axis_command, &value_words, &axis_0, &axis_1,
                  &axis_linear, coordinate_data, &command_words, &int_value,
                  parameter_data, ijk_words);
  if (fail_status != STATUS_GCODE_NO_FAIL)
    return fail_status;

  /* STEP 4: Execute */
  gc_execute_block(axis_command, coordinate_data,
                   parameter_data, axis_0, axis_1, axis_linear, &retval);

  if (fail_status != STATUS_GCODE_NO_FAIL)
    return fail_status;

  return(STATUS_OK|retval);

  // TBD: % to denote start of program. Sets auto cycle start?

}

/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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
#include "serial.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "limits.h"
#include "probe.h"
#include "magazine.h"
#include "motor_driver.h"
#include "report.h"
#include "counters.h"
#include "adc.h"
#include "spi.h"
#include "systick.h"
#include "signals.h"
#include "ad5121.h"
#include "motor_driver.h"
#include "sram.h"

// Declare system global variable structure
system_t sys = {
  .lock_mask = STEPPERS_LONG_LOCK_MASK,
};

#ifndef GRBL_VERSION
  #error "GRBL_VERSION is not defined, please ensure it is defined in the Makefile"
#endif
const char* version_string = "VERSION=" XSTR(GRBL_VERSION);

volatile sys_flags_t sysflags;

void set_overcurrent_retries()
{
  /* Set as output */
  OVERCURRENT_RETRY_DDR |= OVERCURRENT_RETRY_MASK;

  /* Set high */
  OVERCURRENT_RETRY_PORT |= OVERCURRENT_RETRY_MASK;
}

int main(void)
{
  // Ensure that the compiler doesn't try to throw out the version string
  ALWAYS_KEEP(version_string);

  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts

  settings_init(); // Load grbl settings from EEPROM

  /* The ESTOP input is initialized in stepper_init. When we set up digital
  outputs that are connected to the ESTOP, they might get toggled. For safety,
  initialize any outputs connected to the ESTOP after stepper_init. For example:
  the spi driver. */
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt
  counters_init(); // Configure encoder and counter interrupt.
  adc_init();

  set_overcurrent_retries();

  if (settings.use_spi) {
    /* Setup SPI control register and pins */
    spi_init();
    sram_init();
    if (settings.spi_motor_drivers) {
      motor_drv_init();
    }
  }

  SYS_EXEC = 0;   //and mapped port if different

  sys.abort = true;   // Set abort to complete initialization
  sei(); // Enable interrupts

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    linenumber_init();  //reset line numbering buffer
    spindle_init();
    limits_init();
    probe_init();
    magazine_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.
    signals_init();
    systick_init();  // Init systick and systick callbacks

    /* Initialize digital potentiometers */
    if (settings.use_spi && !settings.lc_daughter_card) {
      ad5121_init(AD5121_GAIN);
      ad5121_init(AD5121_CAL);
    }

    // Register first signals update callback
    systick_register_callback(500, signals_callback); // Start polling ADCs 0.5 seconds after init

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Reset system variables.
    sys.abort = false;
    SYS_EXEC = 0;
    if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.flags |= SYSFLAG_AUTOSTART; }
    else { sys.flags &= ~SYSFLAG_AUTOSTART; }

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }
  return 0;   /* Never reached */
}

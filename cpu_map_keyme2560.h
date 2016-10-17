#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

#define GRBL_PLATFORM "KeyMe 2560"

// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// Increase Buffers to make use of extra SRAM
#define RX_BUFFER_SIZE		255
#define TX_BUFFER_SIZE		128
#define BLOCK_BUFFER_SIZE	48
#define LINE_BUFFER_SIZE	100

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR      DDRH
#define STEP_PORT     PORTH
#define STEP_PIN      PINH
#define X_STEP_BIT        0 // Atmega2560 pin 12 / Arduino Digital Pin 17
#define Y_STEP_BIT        1 // Atmega2560 pin 13 / Arduino Digital Pin 16
#define Z_STEP_BIT        2 // Atmega2560 pin 14 / Arduino Digital Pin xx
#define C_STEP_BIT        3 // Atmega2560 pin 15 / Arduino Digital Pin 6
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_DDR      DDRH
#define DIRECTION_PORT     PORTH
#define DIRECTION_PIN      PINH
#define X_DIRECTION_BIT   4 // Atmega2560 pin 16 / Arduino Digital Pin 7
#define Y_DIRECTION_BIT   5 // Atmega2560 pin 17 / Arduino Digital Pin 8
#define Z_DIRECTION_BIT   6 // Atmega2560 pin 18 / Arduino Digital Pin 9
#define C_DIRECTION_BIT   7 // Atmega2560 pin 27 / Arduino Digital Pin xx
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<C_DIRECTION_BIT)) // All direction bits

          // Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_DDR   DDRJ
#define STEPPERS_DISABLE_PORT  PORTJ
#define X_DISABLE_BIT 2  // J2 / Atmega Pin 65
#define Y_DISABLE_BIT 3  // J3 / Atmega Pin 66
#define Z_DISABLE_BIT 4  // J4 / Atmega Pin 67
#define C_DISABLE_BIT 5  // J5 /Atmega Pin 68
#define STEPPERS_DISABLE_MASK ((1<<X_DISABLE_BIT)|(1<<Y_DISABLE_BIT)|(1<<Z_DISABLE_BIT)|(1<<C_DISABLE_BIT))   // All disable bits

#define STEPPERS_LONG_LOCK_MASK  ((1<<Y_DISABLE_BIT)|(1<<Z_DISABLE_BIT))  //Keep gripper and Y engaged longer
#define STEPPERS_LOCK_TIME_MULTIPLE 200  //ms*250 = quarter seconds so 255->63.75s



// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR       DDRD
#define LIMIT_PORT      PORTD
#define LIMIT_PIN       PIND
#define X_LIMIT_BIT     0 // Atmega2560 pin 43 / Arduino Digital Pin 21
#define Y_LIMIT_BIT     1 // Atmega2560 pin 44 / Arduino Digital Pin 20
#define Z_LIMIT_BIT     2 // Atmega2560 pin 45 / Arduino Digital Pin 19
#define C_LIMIT_BIT     3 // Atmega2560 pin 46 / Arduino Digital Pin 18

#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<C_LIMIT_BIT)) // All limit bits
#define HARDSTOP_MASK   ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit that are really hardstops.
#define LIMIT_BIT_SHIFT   0

#define LIMIT_ICR       EICRA       //enables change interrupts
#define LIMIT_INT       (0x55)      //port to monitor (any edge)
#define LIMIT_PCMSK     EIMSK       //Pin change selector
#define LIMIT_ENABLE    LIMIT_MASK
#define LIMIT_INT_vect  INT0_vect
#define LIMIT_INT2_vect INT1_vect

#define TIMING_DDR DDRA
#define TIMING_PORT PORTA
#define TIMING_PIN PINA
#define TIMING_BIT  7
#define TIMING_MASK       (1<<TIMING_BIT) // LED

#ifdef CNC_CONFIGURATION

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // Atmega2560 pin X / Arduino Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // Atmega2560 pin X / Arduino Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // Atmega2560 pin X / Arduino Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // Atmega2560 pin X / Arduino Digital Pin 9
  #endif

#endif

// Define user-control pinouts (cycle start, reset, feed hold) input pins.
// NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
#define PINOUT_DDR       DDRK
#define PINOUT_PIN       PINK
#define PINOUT_PORT      PORTK
#define PIN_RESET        0  // MEGA2560 Analog Pin 8
#define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9
#define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10
#define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
#define PINOUT_INT_vect  PCINT2_vect
#define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
#define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

// Define probe switch input pin.
#define MAGAZINE_ALIGNMENT_DDR       DDRK
#define MAGAZINE_ALIGNMENT_PIN       PINK
#define MAGAZINE_ALIGNMENT_PORT      PORTK
#define MAGAZINE_ALIGNMENT_BIT       3   // Atmega2560 pin 86 // Arduino Analog Pin 11
#define MAGAZINE_ALIGNMENT_MASK      (1 << MAGAZINE_ALIGNMENT_BIT)

//Alias existing probe masks in default Grbl
#define PROBE_DDR       MAGAZINE_ALIGNMENT_DDR
#define PROBE_PIN       MAGAZINE_ALIGNMENT_PIN
#define PROBE_PORT      MAGAZINE_ALIGNMENT_PORT
#define PROBE_MASK      MAGAZINE_ALIGNMENT_MASK

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER2B which is attached to Digital Pin 9
  #define TCCRA_REGISTER		TCCR2A
  #define TCCRB_REGISTER		TCCR2B
  #define OCR_REGISTER		OCR2B

  #define COMB_BIT			COM2B1
  #define WAVE0_REGISTER		WGM20
  #define WAVE1_REGISTER		WGM21
  #define WAVE2_REGISTER		WGM22
  #define WAVE3_REGISTER		WGM23

  #define SPINDLE_PWM_DDR		DDRH
  #define SPINDLE_PWM_PORT    PORTH
  #define SPINDLE_PWM_BIT		6 // Atmega2560 pin X / Arduino Digital Pin 9
#endif // End of VARIABLE_SPINDLE

#define ESTOP_DDR DDRG
#define ESTOP_PORT PORTG
#define ESTOP_PIN PING

#define RUN_ENABLE_BIT  0
#define ESTOP_BIT  2
#define ESTOP_MASK (1<<ESTOP_BIT)

#define MS_DDR DDRC
#define MS_PORT PORTC
#define MS_MASK 0xFF //all 8 bits are used, in XYZC order
#define SET_MICROSTEP(axis,val) (val<<((axis*2)&3)) //2 bits each

#define PFD_DDR DDRL
#define PFD_PORT PORTL
#define PFD_MASK 0xFF //all 8 bits are used, in XYZC order
#define SET_DECAY_MODE(axis,val) (val<<((axis*2)&3)) //2 bits

#define NEW_BOARD
#ifdef NEW_BOARD

#define FDBK_DDR DDRK
#define FDBK_PORT PORTK
#define FDBK_PIN PINK

#define ALIGN_SENSE_BIT 3  //This is also PROBE_BIT

#define FDBK_INT       PCIE2  // Pin change interrupt enable pin
#define FDBK_INT_vect  PCINT2_vect
#define FDBK_PCMSK     PCMSK2 // Pin change interrupt register
#define FDBK_MASK (1<<ALIGN_SENSE_BIT)

// XY or CG Current Control outputs
// TODO: remove this functionality on rev5 and beyond in favor of PWM pin
#define CCTRL_DDR DDRB
#define CCTRL_PORT PORTB
#define CCTRL_CG_BIT 5
#define CCTRL_XY_BIT 4

// Force Sensor Sensitivity Control output
// TODO: remove this functionality on rev5 and beyond in favor of PWM pin
#define FSCTRL_DDR DDRG
#define FSCTRL_PORT PORTG
#define FSCTRL_BIT 5

// Feedback sensor voltage is now analog and called FVOLT
#define FVOLT_DDR DDRK
#define FVOLT_PORT PORTK
#define FVOLT_BIT 7
#define FVOLT_MASK (1<<FVOLT_BIT)

#define FVOLT_ADC 7  // value to write to ADCSRA to read ADC15
#define MUX5_BIT_POS 3
#define MUX5_BIT_VALUE 1 // 1 designates ADC 15

// Measurement of Supply Voltage for all motors, MVOLT
#define MVOLT_DDR DDRF
#define MVOLT_PORT PORTF
#define MVOLT_PIN PINF
#define X_MVOLT_BIT 1
#define Y_MVOLT_BIT 2
#define Z_MVOLT_BIT 3
#define C_MVOLT_BIT 0

#define X_MVOLT_ADC 1
#define Y_MVOLT_ADC 2
#define Z_MVOLT_ADC 3
#define C_MVOLT_ADC 0

#define MVOLT_MASK ((1<<X_MVOLT_BIT)|(1<<Y_MVOLT_BIT)|(1<<Z_MVOLT_BIT)|(1<<C_MVOLT_BIT))

// Define and configure PWM out(s) on timer3 to control the
// force sensor sensitivity and current driving pins (XY and CG)
#define PWM_OUT_TCCRA TCCR3A
#define PWM_OUT_TCCRB TCCR3B
#define PWM_OUT_DDR DDRE
#define PWM_OUT_PORT PORTE
#define PWM_OUT_FSENSE_BIT 4
#define PWM_OUT_XY_CTRL_BIT 5
#define PWM_OUT_CG_CTRL_BIT 3

// IO Reset Functionality
#define IO_RESET_DDR DDRA
#define IO_RESET_PORT PORTA
#define IO_RESET_BIT 0
#define IO_RESET_MASK (1<<IO_RESET_BIT)

#else

#define FDBK_DDR DDRK
#define FDBK_PORT PORTK
#define FDBK_PIN PINK
#define ALIGN_SENSE_BIT 3  //This is also PROBE_BIT
#define Z_ENC_IDX_BIT 4
#define Z_ENC_CHA_BIT 5
#define Z_ENC_CHB_BIT 6

#define FDBK_INT       PCIE2  // Pin change interrupt enable pin
#define FDBK_INT_vect  PCINT2_vect
#define FDBK_PCMSK     PCMSK2 // Pin change interrupt register
#define FDBK_MASK ((1<<Z_ENC_IDX_BIT)|(1<<Z_ENC_CHA_BIT)|(1<<Z_ENC_CHB_BIT)|(1<<ALIGN_SENSE_BIT))

#define MVOLT_DDR DDRD
#define MVOLT_PORT PORTD
#define MVOLT_PIN PIND
#define X_MVOLT_BIT 1
#define Y_MVOLT_BIT 2
#define Z_MVOLT_BIT 3
#define C_MVOLT_BIT 0
#define MVOLT_MASK ((1<<X_MVOLT_BIT)|(1<<Y_MVOLT_BIT)|(1<<Z_MVOLT_BIT)|(1<<C_MVOLT_BIT))

#define IO_RESET_DDR DDRF
#define IO_RESET_PORT PORTF
#define IO_RESET_BIT 0
#define IO_RESET_MASK (1<<IO_RESET_BIT)

#endif


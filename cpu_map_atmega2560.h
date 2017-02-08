#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

#define GRBL_PLATFORM "Atmega2560"
// Define serial port pins and interrupt vectors.
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// Increase Buffers to make use of extra SRAM
//#define RX_BUFFER_SIZE		256
//#define TX_BUFFER_SIZE		128
//#define BLOCK_BUFFER_SIZE	36
//#define LINE_BUFFER_SIZE	100

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR      DDRA
#define STEP_PORT     PORTA
#define STEP_PIN      PINA
#define X_STEP_BIT        0 // MEGA2560 Digital Pin 22
#define Y_STEP_BIT        1 // MEGA2560 Digital Pin 23
#define Z_STEP_BIT        2 // MEGA2560 Digital Pin 24
#define C_STEP_BIT        3 // MEGA2560 Digital Pin 25
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_DDR      DDRA
#define DIRECTION_PORT     PORTA
#define DIRECTION_PIN      PINA
#define X_DIRECTION_BIT   4 // MEGA2560 Digital Pin 26
#define Y_DIRECTION_BIT   5 // MEGA2560 Digital Pin 27
#define Z_DIRECTION_BIT   6 // MEGA2560 Digital Pin 28
#define C_DIRECTION_BIT   7 // MEGA2560 Digital Pin 29
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<C_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_DDR   DDRJ
#define STEPPERS_DISABLE_PORT  PORTJ
#define STEPPERS_DISABLE_BIT   1 // MEGA2560 Digital Pin 14 
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR       DDRB
#define LIMIT_PORT      PORTB
#define LIMIT_PIN       PINB
#define X_LIMIT_BIT     3 // MEGA2560 Digital Pin 50
#define Y_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
#define Z_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
#define C_LIMIT_BIT     6 // MEGA2650 Digital Pin 12
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<C_LIMIT_BIT)) // All limit bits
#define HARDSTOP_MASK   ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit that are really hardstops.
#define LIMIT_BIT_SHIFT 3  // shift right to align with step bits

#define LIMIT_ICR       PCICR       //enables change interrupts
#define LIMIT_INT       (1<<PCIE0)  //port to monitor
#define LIMIT_PCMSK     PCMSK0      //Pin change selector
#define LIMIT_ENABLE    LIMIT_MASK
#define LIMIT_INT_vect   PCINT0_vect //change handler


#define TIMING_DDR DDRB
#define TIMING_PORT PORTB
#define TIMING_ENABLE_BIT  7
#define TIMING_MASK        (1<<TIMING_ENABLE_BIT) // LED

#define CNC_CONFIGURATION

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR   DDRH
#define SPINDLE_ENABLE_PORT  PORTH
#define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6
#define SPINDLE_DIRECTION_DDR   DDRE
#define SPINDLE_DIRECTION_PORT  PORTE
#define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

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
#define PROBE_DDR       DDRK
#define PROBE_PIN       PINK
#define PROBE_PORT      PORTK
#define PROBE_BIT       3  // MEGA2560 Analog Pin 11
#define PROBE_MASK      (1<<PROBE_BIT)

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
  #define SPINDLE_PWM_BIT		6 // MEGA2560 Digital Pin 9
#endif // End of VARIABLE_SPINDLE

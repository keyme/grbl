#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

#error "Not Supported"

#define GRBL_PLATFORM "Atmega328p"
// Define serial port pins and interrupt vectors.
#define SERIAL_RX     USART_RX_vect
#define SERIAL_UDRE   USART_UDRE_vect

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR        DDRD
#define STEP_PORT       PORTD
#define X_STEP_BIT      2  // Uno Digital Pin 2
#define Y_STEP_BIT      3  // Uno Digital Pin 3
#define Z_STEP_BIT      4  // Uno Digital Pin 4
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_DDR     DDRD
#define DIRECTION_PORT    PORTD
#define X_DIRECTION_BIT   5  // Uno Digital Pin 5
#define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
#define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_DDR    DDRB
#define STEPPERS_DISABLE_PORT   PORTB
#define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR        DDRB
#define LIMIT_PIN        PINB
#define LIMIT_PORT       PORTB
#define X_LIMIT_BIT      1  // Uno Digital Pin 9
#define Y_LIMIT_BIT      2  // Uno Digital Pin 10
#ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
#else
  #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
#endif
#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
#define HARDSTOP_MASK    ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All true hardstops
#define LIMIT_BIT_SHIFT  -1      //shift limit pins 1 left to align w/ step bits.
                                 // won't work with variable spindle, unless we move limits to 10,11,12 and
                                 // put spindle on pwm 9, leave dir on 13.  (then shift=0)


#define LIMIT_ICR       PCICR       //enables change interrupts
#define LIMIT_INT       (1<<PCIE0)  //port to monitor
#define LIMIT_PCMSK     PCMSK0      //Pin change selector
#define LIMIT_ENABLE    LIMIT_MASK
#define LIMIT_INT_vect   PCINT0_vect //change handler


#define TIMING_DDR DDRB
#define TIMING_PORT PORTB
#define TIMING_ENABLE_BIT  5 //Uno Digital Pin 13
#define TIMING_MASK        (1<<TIMING_ENABLE_BIT) // LED
 
#define CNC_CONFIGURATION 

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR    DDRB
#define SPINDLE_ENABLE_PORT   PORTB
#ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
#else
  #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
#endif  
#define SPINDLE_DIRECTION_DDR   DDRB
#define SPINDLE_DIRECTION_PORT  PORTB
#define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

// Define user-control pinouts (cycle start, reset, feed hold) input pins.
// NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
#define PINOUT_DDR       DDRC
#define PINOUT_PIN       PINC
#define PINOUT_PORT      PORTC
#define PIN_RESET        0  // Uno Analog Pin 0
#define PIN_FEED_HOLD    1  // Uno Analog Pin 1
#define PIN_CYCLE_START  2  // Uno Analog Pin 2
#define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
#define PINOUT_INT_vect  PCINT1_vect
#define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
#define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  
// Define probe switch input pin.
#define PROBE_DDR       DDRC
#define PROBE_PIN       PINC
#define PROBE_PORT      PORTC
#define PROBE_BIT       5  // Uno Analog Pin 5
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
// Advanced Configuration Below You should not need to touch these variables
  #define TCCRA_REGISTER	 TCCR2A
  #define TCCRB_REGISTER	 TCCR2B
  #define OCR_REGISTER     OCR2A

  #define COMB_BIT	     COM2A1
  #define WAVE0_REGISTER	 WGM20
  #define WAVE1_REGISTER	 WGM21
  #define WAVE2_REGISTER	 WGM22
  #define WAVE3_REGISTER	 WGM23

  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
  #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
  #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
  #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
#endif // End of VARIABLE_SPINDLE

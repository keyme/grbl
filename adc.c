
#include "system.h"
#include "adc.h"
#include "nuts_bolts.h"
#include "settings.h"

#define ADMUX_SELECTION_MASK    0x7
#define MUX5_MASK               0x8

channel_t active_channel = 0;

uint8_t channel_map[VOLTAGE_SENSOR_COUNT] = {
    X_ADC,
    Y_ADC,
    Z_ADC,
    C_ADC,
    0,  // This value is initialized adc_init
    RD_ADC
};

void setup_adc_channel(uint8_t channel) 
{
  // Disable ADC Interrupt
  bit_false(ADCSRA, 1 << ACIE);

  // Stop ADC conversion
  bit_false(ADCSRA, 1 << ADSC);

  // Select Channel - ADCSRB, MUX5
  // If the channel number is higher than 7, the MUX5 bit in ADCSRB needs to be set  
  if (channel & MUX5_MASK) {
    bit_true(ADCSRB, (1 << MUX5_BIT_POS));
  } else {
    bit_false(ADCSRB, (1 << MUX5_BIT_POS));
  }

  // Clear the previously selected channel
  bit_false(ADMUX, (ADMUX_SELECTION_MASK));

  // Select Channel
  bit_true(ADMUX, (channel & ADMUX_SELECTION_MASK));

  // Enable ADC, Prescaler: 128x
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Enable ADC Interrupt
  bit_true(ADCSRA, 1 << ACIE);

  // Start ADC conversion
  bit_true(ADCSRA, 1 << ADSC);

}

ISR(ADC_vect)
{
  // Store raw ADC reading
  raw_adc_readings[active_channel] = ADC;
  
  // Update the active channel
  active_channel = (active_channel == REV) ? X : active_channel + 1;
  
  // Setup the ADC to read from the next channel and
  // trigger an interupt once ADC conversion is completed
  setup_adc_channel(channel_map[active_channel]);

}

void adc_init()
{

  // These pins are initialized here because there are 
  // no other where the pins can be initialized with the
  // associated peripherals, such as with the stepper motors.


  if (settings.lc_daughter_card) {
    // Daughter Card load cell conditioner
    LC_DDR &= ~(LC_MASK);
    channel_map[FORCE] = LC_ADC;
  } else {
    // Onboard load cell conditioner
    ONBOARD_LC_DDR &= ~(ONBOARD_LC_MASK);
    channel_map[FORCE] = F_ADC;
  }

  // Set revision divider pin as an input
  RD_DDR &= ~(RD_MASK);

  // Initialize ADCSRA to 0x00
  ADCSRA = 0x00;

  // Set ADC reference
  ADMUX |= (1 << REFS0);

  // Start cycling through the ADC channels
  active_channel = X;
  setup_adc_channel(active_channel);
 
  // Enable global interrupts
  sei(); 

}



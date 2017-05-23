#include <stdio.h>

#include "motor_driver.h"
#include "nuts_bolts.h"
#include "report.h"
#include "spi.h"

#define ADDRESS_IDX     4U
#define ADDRESS_MASK    0x70

#define DECMOD_MASK     0x7
#define DECMOD_IDX      8U

#define TORQUE_MASK     0xFF
#define TORQUE_IDX      0

#define ISGAIN_MASK     0x3
#define ISGAIN_IDX      8U

#define ENABLE_MASK     0x1
#define ENABLE_IDX      0

#define STEPS_MASK      0x0F
#define STEPS_IDX       3U

#define REG_READ        0x80

const char * reg_names[] = {"CTRL", "TORQUE", "OFF", "BLANK",
                            "DECAY", "STALL", "DRIVE", "STATUS"};

void _motor_drv_write_reg(enum stepper_e stepper, enum address_e address, uint8_t * data)
{
  uint8_t data_out[2] = {(address << ADDRESS_IDX) | (data[1] & 0x0F), data[0]};

  bit_true(SCS_PORT, 1 << scs_pin_lookup[stepper]);
  spi_write(data_out, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[stepper]);

}

void _motor_drv_read_reg(enum stepper_e stepper,
                         enum address_e address,
                         uint8_t * data)
{
  uint8_t data_out[2] = {REG_READ | (address << ADDRESS_IDX), 0};

  bit_true(SCS_PORT, 1 << scs_pin_lookup[stepper]);
  spi_transact_array(data_out, data, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[stepper]);

  data[0] &= ~(REG_READ | ADDRESS_MASK);
}

void _motor_drv_set_val(enum stepper_e stepper,
                        enum address_e address,
                        uint8_t idx,
                        uint8_t mask,
                        uint8_t val)
{
  uint8_t data_array[] = {0, 0};
  uint16_t data = 0;
  /* Read the register */
  _motor_drv_read_reg(stepper, address, data_array);

  data = data_array[0] | (data_array[1] << 8);

  /* Clear the bits that need to be set */
  data &= (mask << idx);

  /* Set the new value */
  data |= (val & mask) << idx;

  data_array[0] = (data & 0xFF00) >> 8;
  data_array[1] = data & 0xFF;

  /* Clear the bits in the address bits space */
  data_array[1] |= ~ADDRESS_MASK;

  /* Write the updated value to the register */
  _motor_drv_write_reg(stepper, address, data_array);

}

void motor_drv_report_register_vals(enum stepper_e stepper)
{
  #ifdef DEBUG
  for (int idx = 0; idx <= STATUS; idx++) {
    char dbg_msg[30];
    uint8_t data[2];
    _motor_drv_read_reg(stepper, (enum address_e)idx, data);
    snprintf(dbg_msg, 30, "%d %s MSB: %d, LSB: %d", idx, reg_names[idx],
            data[0], data[1]);
    report_debug_message(dbg_msg);
  }
  #else
    (void)stepper;
  #endif
}

void motor_drv_set_decay_mode(enum stepper_e stepper, enum decmod_e decmod)
{
  _motor_drv_set_val(stepper, DECAY, DECMOD_IDX, DECMOD_MASK, decmod);
}

void motor_drv_set_torque(enum stepper_e stepper, uint8_t torque)
{
  _motor_drv_set_val(stepper, TORQUE, TORQUE_IDX, TORQUE_MASK, torque);
}

void motor_drv_set_isgain(enum stepper_e stepper, enum isgain_e isgain)
{
  _motor_drv_set_val(stepper, CTRL, ISGAIN_IDX, ISGAIN_MASK, isgain);
}

void motor_drv_set_micro_stepping(enum stepper_e stepper, enum steps_e steps)
{
  _motor_drv_set_val(stepper, CTRL, STEPS_IDX, STEPS_MASK, steps);
}

void motor_drv_enable_motor(enum stepper_e stepper)
{
  _motor_drv_set_val(stepper, CTRL, ENABLE_IDX, ENABLE_MASK, 1);
}

void motor_drv_disable_motor(enum stepper_e stepper)
{
  _motor_drv_set_val(stepper, CTRL, ENABLE_IDX, ENABLE_MASK, 0);
}

void motor_drv_init()
{
  /* Configure reset pin as output */
  MOTOR_RESET_DDR |= ((1 << MOTOR_RESET_PIN) | (1 << MOTOR_RESET_LINE_DRIVER_PIN));

  /* Set line driver /OE to low */
  MOTOR_RESET_PORT &= ~(1 << MOTOR_RESET_LINE_DRIVER_PIN);

  /* Toggle the motor reset pin */
  MOTOR_RESET_PORT &= ~(1 << MOTOR_RESET_PIN);
  delay_ms(1);
  MOTOR_RESET_PORT |= (1 << MOTOR_RESET_PIN);
  delay_ms(1);
  MOTOR_RESET_PORT &= ~(1 << MOTOR_RESET_PIN);
  delay_ms(1);
}

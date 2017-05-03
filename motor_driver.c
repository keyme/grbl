#include <stdio.h>

#include "motor_driver.h"
#include "nuts_bolts.h"
#include "spi.h"

#define ADDRESS_IDX 5U

#define DECMOD_MASK     0x0F
#define DECMOD_IDX      8U

#define TORQUE_MASK      0xFF
#define TORQUE_IDX       0

#define ENABLE_MASK     0x1
#define ENABLE_IDX      0

#define STEPS_MASK      0x0F
#define STEPS_IDX       3U

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
  uint8_t data_out[2] = {address << ADDRESS_IDX, 0};
  bit_true(SCS_PORT, 1 << scs_pin_lookup[stepper]);
  spi_transact_array(data_out, data, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[stepper]);
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

  /* Write the updated value to the register */
  _motor_drv_write_reg(stepper, address, data_array);
}

void motor_drv_set_decay_mode(enum stepper_e stepper, enum decmod_e decmod)
{
  _motor_drv_set_val(stepper, DECAY, DECMOD_IDX, DECMOD_MASK, decmod);
}

void motor_drv_set_torque(enum stepper_e stepper, uint8_t torque)
{
  _motor_drv_set_val(stepper, TORQUE, TORQUE_IDX, TORQUE_MASK, torque);
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

#include <stdio.h>

#include "motor_driver.h"
#include "nuts_bolts.h"
#include "report.h"
#include "spi.h"
#include "settings.h"

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

#define RDIR_MASK       0x1
#define RDIR_IDX        1

#define STEPS_MASK      0xF
#define STEPS_IDX       3U

#define REG_RW          0x80

#define TORQUE_VAL_5A   150U
#define TORQUE_VAL_3A   90U

/* Note, the current allowed through the X, Y and C motors
   (chopping current) is calculated as:

      I = (2.75V * TORQUE) / (256 * ISGAIN * RISENSE)

    with TORQUE = 150, ISGAIN = 5 and RISENSE = 0.065 Ohm,

      I = 4.95 A

    For the gripper, TORQUE = 90, ISGAIN = 5

      I = 2.97
*/


const char * reg_names[] = {"CTRL", "TORQUE", "OFF", "BLANK",
                            "DECAY", "STALL", "DRIVE", "STATUS"};

void _motor_drv_write_reg(enum stepper_e stepper, enum address_e address, uint16_t data)
{
  /* Write to the specified address of stepper. The 12 least significant
  bits are data bits to be written into the register specified by address.
  The 4 most significant bits are masked with the RW bit and address*/
  spi_set_mode(0, 0);

  uint8_t data_out[2] = {(address << ADDRESS_IDX) | ((data & 0x0F00) >> 8),
                         data & 0x00FF};
  uint8_t data_in[2];

  bit_true(SCS_PORT, 1 << scs_pin_lookup[stepper]);
  spi_transact_array(data_out, data_in, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[stepper]);

}

uint16_t _motor_drv_read_reg(enum stepper_e stepper, enum address_e address)
{
  spi_set_mode(0, 0);

  uint8_t data_out[2] = {REG_RW | (address << ADDRESS_IDX), 0};
  uint8_t data_in[2];

  bit_true(SCS_PORT, 1 << scs_pin_lookup[stepper]);
  spi_transact_array(data_out, data_in, 2);
  bit_false(SCS_PORT, 1 << scs_pin_lookup[stepper]);

  data_in[0] &= ~(REG_RW | ADDRESS_MASK);

  return ((data_in[0] << 8) | data_in[1]);
}

void _motor_drv_set_val(enum stepper_e stepper,
                        enum address_e address,
                        uint8_t idx,
                        uint16_t mask,
                        uint16_t val)
{
  /* Read-modify-write the specified address */

  /* Read the register */
  uint16_t data;
  data = _motor_drv_read_reg(stepper, address);

  /* Clear the bits that need to be set */
  data &= ~(mask << idx);

  /* Set the new value */
  data |= (val & mask) << idx;

  /* Write the updated value to the register */
  _motor_drv_write_reg(stepper, address, data);

}

void motor_drv_report_register_vals(enum stepper_e stepper)
{
  #ifdef DEBUG
  for (int idx = 0; idx <= STATUS; idx++) {
    char dbg_msg[30];
    uint16_t data;
    data = _motor_drv_read_reg(stepper, (enum address_e)idx);
    snprintf(dbg_msg, 30, "%d %s MSB: %d, LSB: %d", idx, reg_names[idx],
             ((data & 0xFF00) >> 8), (data & 0x00FF));
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

void motor_drv_set_micro_steps(enum stepper_e stepper, enum steps_e steps)
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

void motor_drv_inverse_dir(enum stepper_e stepper, bool reversed)
{
  if (reversed) {
    _motor_drv_set_val(stepper, CTRL, RDIR_IDX, RDIR_MASK, 1);
  } else {
    _motor_drv_set_val(stepper, CTRL, RDIR_IDX, RDIR_MASK, 0);
  }
}

uint8_t _motor_drv_get_micro_steps_mask(enum stepper_e idx)
{
  return (settings.microsteps & (0x3 << (2 * idx))) >> (2 * idx);
}

void motor_drv_init()
{
  /* Note that this function is called everytime one of the
  microstepping values in the settings struct is changed over
  serial. */

  /* Wake up motor drivers before issuing a reset */
  STEPPERS_DISABLE_PORT |= STEPPERS_DISABLE_MASK;

  /* Configure reset pin as output */
  MOTOR_RESET_DDR |= (1 << MOTOR_RESET_PIN);

  /* Toggle the motor reset pin */
  MOTOR_RESET_PORT &= ~(1 << MOTOR_RESET_PIN);
  delay_ms(1);
  MOTOR_RESET_PORT |= (1 << MOTOR_RESET_PIN);
  delay_ms(1);
  MOTOR_RESET_PORT &= ~(1 << MOTOR_RESET_PIN);
  delay_ms(1);

  /* Note that the X, Y and C motors are rated for
  5A, but the gripper motor is only rated for 3A */
  motor_drv_set_torque(XTABLE, TORQUE_VAL_3A);
  motor_drv_set_isgain(XTABLE, FIVE);

  motor_drv_set_torque(YTABLE, TORQUE_VAL_3A);
  motor_drv_set_isgain(YTABLE, FIVE);

  motor_drv_set_torque(CAROUSEL, TORQUE_VAL_3A);
  motor_drv_set_isgain(CAROUSEL, FIVE);

  motor_drv_set_torque(GRIPPER, TORQUE_VAL_3A);
  motor_drv_set_isgain(GRIPPER, FIVE);

  for (int idx = 0; idx < 4; idx++) {
    /* Set microstepping */
    uint8_t steps = *(&settings.x_microsteps + idx);

    motor_drv_set_micro_steps((enum stepper_e)idx, (enum steps_e)steps);
    motor_drv_enable_motor((enum stepper_e)idx);
  }
}

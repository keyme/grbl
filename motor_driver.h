#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "system.h"

enum address_e {
  CTRL = 0,
  TORQUE,
  OFF,
  BLANK,
  DECAY,
  STALL,
  DRIVE,
  STATUS
};

enum steps_e {
  FULL = 0,
  HALF,
  QUARTER,
  EIGHTH,
  SIXTEENTH,
  THIRTY_SECOND,
  SIXTY_FOURTH,
  ONE_TWENTY_EIGHTH
};

enum isgain_e {
  FIVE = 0,
  TEN,
  TWENTY,
  FORTY 
};

enum stepper_e {
  XTABLE = 0,
  YTABLE,
  GRIPPER,
  CAROUSEL
};

/* The order of the entries in this enum is important */
enum decmod_e {
  SLOW = 0,
  SLOW_INCR_MIXED_DECR,
  FAST,
  MIXED,
  SLOW_INCR_AUTO_MIXED_DECR,
  AUTO_MIXED
};

static const uint8_t scs_pin_lookup[4] = {
  SCS_XTABLE_PIN,
  SCS_YTABLE_PIN,
  SCS_GRIPPER_PIN,
  SCS_CAROUSEL_PIN
};

void motor_drv_set_decay_mode(enum stepper_e stepper, enum decmod_e decmod);
void motor_drv_set_torque(enum stepper_e stepper, uint8_t torque);
void motor_drv_set_isgain(enum stepper_e stepper, enum isgain_e isgain);
void motor_drv_set_micro_stepping(enum stepper_e stepper, enum steps_e steps);
void motor_drv_enable_motor(enum stepper_e stepper);
void motor_drv_disable_motor(enum stepper_e stepper);

#endif //MOTOR_DRIVER_H

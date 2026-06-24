#ifndef ejector_stepper_h
#define ejector_stepper_h

#include "system.h"

void ejector_stepper_init(void);
void ejector_stepper_service(void);
uint8_t ejector_stepper_execute_line(char *line, uint8_t *char_counter);
bool ejector_stepper_is_active(void);

#endif

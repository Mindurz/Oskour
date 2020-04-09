
#ifndef MOTORS_CONTROL_H_
#define MOTORS_CONTROL_H_

#include <stdint.h>
#include <hal.h>

#define NORMAL_SPEED 600

enum positions {POSITION_NOT_REACHED, POSITION_REACHED };

void motors_control_init(void);

void motors_set_position(int32_t position_r, int32_t position_l, int16_t speed_r, int16_t speed_l);

uint8_t motors_get_reached(void);

void motors_stop(void);

void motors_update_target_reached(void);

#endif /* MOTORS_CONTROL_H_ */

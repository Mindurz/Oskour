#ifndef MOTORS_CONTROL_H_
#define MOTORS_CONTROL_H_

#include <stdint.h>
#include <hal.h>

#define NORMAL_SPEED 600
#define TURN_SPEED 400

enum positions {
	TARGET_NOT_REACHED,
	TARGET_REACHED
};

void motors_update_target_reached(void);
void motors_set_target(int32_t position_r, int32_t position_l, int16_t speed_r, int16_t speed_l);
uint8_t motors_is_target_reached(void);
void motors_stop(void);

#endif /* MOTORS_CONTROL_H_ */

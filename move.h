#ifndef MOVE_H_
#define MOVE_H_
#define PI 3.141592

#define WHEEL_DISTANCE  	5.6f	//cm
#define PERIMETER_EPUCK 	PI * WHEEL_DISTANCE
#define ANGLE(angle) (PERIMETER_EPUCK*angle)/360

enum state_t {
	MOVING = 0,
	WAITING,
	TURNING
};

void navigation_start(void);
uint8_t is_wall_in_front(float left_read, float right_read);
uint8_t is_wall_on_right(float read);
uint8_t is_wall_on_left(float read);

#endif /* CAMREG_EDGEGARD_H_ */

#ifndef MOVE_H_
#define MOVE_H_
#define PI 3.141592

#define WHEEL_DISTANCE  	5.6f	//cm
#define PERIMETER_EPUCK 	PI * WHEEL_DISTANCE
#define ANGLE(angle) (PERIMETER_EPUCK*angle)/360

enum state_t {
	MOVING = 0,
	WAITING,
	TURNING,
	STARTING
};

enum moving_along_t {
	X_POS = 0,
	Y_POS,
	X_NEG,
	Y_NEG
};

void navigation_start(void);
uint8_t is_wall(float read);
uint8_t is_path_open(float read);
float trajectory_correction(float read);
uint8_t get_has_turned(void);
void set_has_turned(uint8_t setter);
void rotation_update(int rotation);
void position_update(void);
void switch_state(uint8_t new_state);


#endif /* CAMREG_EDGEGARD_H_ */

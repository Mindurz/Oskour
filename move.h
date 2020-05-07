#ifndef MOVE_H_
#define MOVE_H_

enum state_t {
	MOVING = 0,
	WALL_IN_FRONT,
	TURNING,
	STARTING,
	JUNCTION,
	FINISH
};

enum moving_along_t {
	X_POS = 0,
	Y_POS,
	X_NEG,
	Y_NEG
};

enum turn_type_t {
	LEFT_TURN = -1,
	RIGHT_TURN = 1,
	U_TURN = 2
};

void navigation_start(void);
void proximity_measure(uint8_t nb_iteration);
uint8_t is_wall(float read);
uint8_t is_path_open(float read);
float trajectory_correction(float read_right, float read_right_49, float read_left, float read_left_49);
uint8_t get_has_turned(void);
void set_has_turned(uint8_t setter);
void rotation_update(int rotation);
void position_update(int offset_right, int offset_left);
uint8_t get_state(void);
void send_position_to_computer(uint8_t pos_x, uint8_t pos_y,uint8_t line, uint8_t sign_x, uint8_t sign_y);
int8_t convert_position_to_duplo(int16_t pos);
void Vetterli_tango(void);

#endif /* CAMREG_EDGEGARD_H_ */

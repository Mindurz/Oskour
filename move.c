#include "ch.h"
#include "hal.h"
#include <math.h>

#include <motors_control.h>

#include <sensors/proximity.h>
#include <motors.h>
#include "leds.h"
#include "move.h"
#include <main.h>
#include <process_image.h>
#include <audio/play_melody.h>

#define LEFT_IR_SENSOR 5
#define RIGHT_IR_SENSOR 2
#define RIGHT_FRONT_17_IR_SENSOR 0
#define LEFT_FRONT_17_IR_SENSOR 7
#define LEFT_FRONT_49_IR_SENSOR 6
#define RIGHT_FRONT_49_IR_SENSOR 1

#define PROXIMITY_THRESHOLD_FRONT 220 // 2cm
#define PROXIMITY_OPEN_THRESHOLD 80 // ~3.5cm

#define NB_STEP_QUARTER_TURN   320
#define NB_STEP_U_TURN   650
#define NB_STEP_OPENING 250 //The robot advances 3.25cm when it sees an opening on its right

#define LEFT_PROX_OFFSET 20 // Equalize the values of the left sensor for the trajectory correction (moyenne approximative)
#define KP 0.02
#define KP_WEIGHT_49 2
#define KP_WEIGHT_SIDE 0.5

#define ASCII_VALUE_OF_1 49

#define DRIFT_CORRECTION 0.1 //Estimation that there will 10% more steps on every straight line
#define ODOMETRY_MARGIN 150 //margin to detect if the robot is back at the starting point.
#define STEP_TO_LEGO 0.00866 // 0.013/1.5

#define FINISH_MESSAGE_KEY 64

static uint8_t current_state = STARTING ;
static uint8_t cmd = 0;
static int8_t current_direction = X_POS;
static int16_t nb_step_start_line_right = 0;
static int16_t nb_step_start_line_left = 0;
static int16_t step_before_turn_right = 0;
static int16_t step_before_turn_left = 0;
static int16_t current_xpos = 0;
static int16_t current_ypos = 0;

static uint8_t finished = 0;
static uint8_t tango_offset= 0;

static float proximity_read_left = 0;
static float proximity_read_right = 0;
static float proximity_read_left_front_49 = 0;
static float proximity_read_right_front_49 = 0;
static float proximity_read_left_front_17 = 0;
static float proximity_read_right_front_17 =0;

uint8_t has_turned = 1; //variable to not take two pictures of the same wall

/*
 * Those variables ensure that the robot will not turn right after a turn.
 * They allow to check if the robot has crossed the detected opening.
 * Note that those applies only if the robot tries to turn at a point
 * where there is no wall in front of it.
 */
static uint8_t wall_on_right = 0 ;
static uint8_t free_to_turn = 0 ;

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){

		time = chVTGetSystemTime();
//measure of the proximity sensors
		proximity_read_left = 0;
		proximity_read_right = 0;
		proximity_read_left_front_49 = 0;
		proximity_read_right_front_49 = 0;
		proximity_read_left_front_17 = 0;
		proximity_read_right_front_17 =0;
		proximity_measure(5);

/*FINITE STATE MACHINE
 * The e-puck will use the right-hand method, it turns right every time it can */
		switch(current_state){

			case WALL_IN_FRONT:

				position_update(step_before_turn_right,step_before_turn_left);

				if(is_path_open(proximity_read_right)){
					rotation_update(RIGHT_TURN);
					motors_set_target(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);

				}else if(is_path_open(proximity_read_left)) {
					rotation_update(LEFT_TURN);
					motors_set_target(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, TURN_SPEED, -TURN_SPEED);

				}else{
					rotation_update(U_TURN);
					motors_set_target(NB_STEP_U_TURN, NB_STEP_U_TURN, -TURN_SPEED, TURN_SPEED);
				}
				current_state = TURNING;
				free_to_turn = 0;
				break;

			case MOVING:

				if(nb_step_start_line_right == 0 && nb_step_start_line_left == 0){
					nb_step_start_line_right = right_motor_get_pos() ;
					nb_step_start_line_left = left_motor_get_pos();
				}

				if(is_wall(proximity_read_left_front_17) && is_wall(proximity_read_right_front_17)) {
					motors_stop();
					current_state = WALL_IN_FRONT ;

				}else if(is_path_open(proximity_read_right) && free_to_turn){

					step_before_turn_right = right_motor_get_pos() ;
					step_before_turn_left = left_motor_get_pos();
					motors_set_target(NB_STEP_OPENING, NB_STEP_OPENING, NORMAL_SPEED, NORMAL_SPEED);
					current_state = JUNCTION ;
					free_to_turn = 0;
					wall_on_right = 0;

				}else{
					float correction = trajectory_correction(proximity_read_right, proximity_read_right_front_49, proximity_read_left, proximity_read_left_front_49);
					right_motor_set_speed(NORMAL_SPEED - correction);
					left_motor_set_speed(NORMAL_SPEED + correction);
				}
				if(!free_to_turn) {
					if(is_wall(proximity_read_right))	{
						wall_on_right = 1;
					}
					if(wall_on_right && is_path_open(proximity_read_right)) {
						free_to_turn = 1;
						wall_on_right = 0;
					}
				}
				break;

			case TURNING:

				motors_update_target_reached();

				if(motors_is_target_reached() == 1) {
					left_motor_set_speed(NORMAL_SPEED);
					right_motor_set_speed(NORMAL_SPEED);
					has_turned = 1 ;
					nb_step_start_line_right = 0;
					nb_step_start_line_left = 0;
					current_state = MOVING;
				}

				if(abs(current_xpos) < ODOMETRY_MARGIN && abs(current_ypos) < ODOMETRY_MARGIN) {
					current_state = FINISH;
				}
				break;

			case JUNCTION:

				motors_update_target_reached();

				if(motors_is_target_reached() == 1) {
					position_update(step_before_turn_right,step_before_turn_left);
					step_before_turn_right = 0;
					step_before_turn_left = 0;
					rotation_update(1);
					motors_set_target(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
				}
				break;


			case STARTING:

				cmd = chSequentialStreamGet((BaseSequentialStream *) &SD3);
	    		if(cmd == ASCII_VALUE_OF_1) {
	    			current_state = MOVING;
	    		}
				break;

			case FINISH:

				Vetterli_tango();
				if(!finished) {
					send_position_to_computer(0, 0, FINISH_MESSAGE_KEY,0,0);
					finished = 1;
				}
				playMelody(2,0,0);
				chThdSleepMilliseconds(500);
				break;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

//This function allow to take the mean of the wanted number of measurements in order to have more stable values*/
void proximity_measure(uint8_t nb_iteration) {

	for(int i = 0 ; i < nb_iteration ; i++) {
		proximity_read_left += get_prox(LEFT_IR_SENSOR);
		proximity_read_right += get_prox(RIGHT_IR_SENSOR);
		proximity_read_left_front_49 += get_prox(LEFT_FRONT_49_IR_SENSOR);
		proximity_read_right_front_49 += get_prox(RIGHT_FRONT_49_IR_SENSOR);
		proximity_read_left_front_17 += get_prox(LEFT_FRONT_17_IR_SENSOR);
		proximity_read_right_front_17 += get_prox(RIGHT_FRONT_17_IR_SENSOR);
	}
	proximity_read_left /= nb_iteration;
	proximity_read_right /= nb_iteration;
	proximity_read_left_front_49 /= nb_iteration;
	proximity_read_right_front_49 /= nb_iteration;
	proximity_read_left_front_17 /= nb_iteration;
	proximity_read_right_front_17 /= nb_iteration;
}

uint8_t is_wall(float read) {
	return (read > PROXIMITY_THRESHOLD_FRONT);
}

uint8_t is_path_open(float read) {
	return (read < PROXIMITY_OPEN_THRESHOLD);
}

uint8_t get_has_turned() {
	return has_turned;
}

void set_has_turned(uint8_t setter) {
	has_turned = setter;
}

float trajectory_correction(float read_right, float read_right_49, float read_left, float read_left_49) {

	float error = 0;
	error = ((KP_WEIGHT_SIDE*read_left + KP_WEIGHT_49*(read_left_49+LEFT_PROX_OFFSET)) - (KP_WEIGHT_SIDE*read_right + KP_WEIGHT_49*read_right_49));
	return KP*error;
}

/*rotation takes -1,0,1,2 for values
	-1 => 90 Turn to the left
	0 => No turn
	1 => 90 Turn to the right
	2 => U turn
*/
void rotation_update(int rotation) {

	current_direction = current_direction + rotation;
	if(current_direction < 0){
		current_direction += 4;
	}else if(current_direction >= 4) {
		current_direction -= 4;
	}
}

void position_update(int offset_right, int offset_left){

	int32_t delta_steps;
	delta_steps = (abs(right_motor_get_pos() - nb_step_start_line_right + offset_right) + abs(left_motor_get_pos() -nb_step_start_line_left + offset_left))/2;
	delta_steps -= DRIFT_CORRECTION*delta_steps;
	switch(current_direction){

		case X_POS:
			current_xpos += delta_steps;
			break;

		case Y_POS:
			current_ypos += delta_steps;
			break;

		case X_NEG:
			current_xpos -= delta_steps;
			break;

		case Y_NEG:
			current_ypos -= delta_steps;
			break;
	}
	uint8_t sign_x = 0;
	uint8_t sign_y = 0;
	if(current_xpos < 0){
		sign_x = 1;
	}
	if(current_ypos < 0){
		sign_y = 1;
	}
	send_position_to_computer(abs(convert_position_to_duplo(current_xpos)), abs(convert_position_to_duplo(current_ypos)),get_nb_line(),sign_x,sign_y);
	set_nb_line(0);
}

uint8_t get_state() {
	return current_state;
}

void switch_state(uint8_t new_state) {
	current_state = new_state;
}

int8_t convert_position_to_duplo(int16_t pos) {
	return floor(STEP_TO_LEGO*pos);
}

void send_position_to_computer(uint8_t pos_x, uint8_t pos_y, uint8_t line, uint8_t sign_x, uint8_t sign_y) {
	uint8_t data[] = {pos_x,pos_y,line,sign_x,sign_y};
	int8_t size = sizeof(data);
	SendUint8ToComputer(data, size);
}

/* Demarrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO, Navigation, NULL);
}

void Vetterli_tango(void){

	if(!finished){
		left_motor_set_speed(NORMAL_SPEED);
		right_motor_set_speed(-NORMAL_SPEED);
	}

	for(uint8_t i= 0; i<NUM_RGB_LED; i++){
		for(uint8_t j = 0; j<NUM_COLOR_LED;j++){
			switch((i+tango_offset)%4){
				case 0:
					switch(j){
						case 0:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
						case 1:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY/10);
							break;
						case 2:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
					}
					break;

				case 1:
					switch(j){
						case 0:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY/10);
							break;
						case 1:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
						case 2:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY/10);
							break;
					}
					break;

				case 2:
					switch(j){
						case 0:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY/10);
							break;
						case 1:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
						case 2:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
					}
					break;

				case 3:
					switch(j){
						case 0:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
						case 1:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY);
							break;
						case 2:
							toggle_rgb_led(i,j,RGB_MAX_INTENSITY/10);
							break;
					}
					break;
			}
		}
	}
	tango_offset = tango_offset + 1;
}

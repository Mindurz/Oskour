#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <motors_control.h>
#include <sensors/proximity.h>
#include <motors.h>
#include "leds.h"
#include "move.h"
#include <main.h>
#include <process_image.h>
#include <audio/play_melody.h>

//#define NB_PROX_SENSORS 8 a taej si sa marche sans
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
#define NB_STEP_OPENING 250 //le robot avance de 3.25cm avant de s'engager dans une ouverture sur sa droite

#define LEFT_PROX_OFFSET 20 // Egalise les valeurs entre capteur gauche et droite pour la correction de trajectoire (moyenne approximative)
#define KP 0.02
#define DRIFT_CORRECTION 0.1 //Considération de 10% de steps en trop dans dans toute ligne droite

#define ASCII_VALUE_OF_1 49

static uint8_t current_state = STARTING ;
static uint8_t cmd = 0;
static int8_t current_direction = X_POS;
static int16_t nb_step_start_line_right = 0;
static int16_t nb_step_start_line_left = 0;
static int16_t step_before_turn_right = 0;
static int16_t step_before_turn_left = 0;
static int16_t current_xpos = 0;
static int16_t current_ypos = 0;

static float proximity_read_left = 0;
static float proximity_read_right = 0;
static float proximity_read_left_front_49 = 0;
static float proximity_read_right_front_49 = 0;
static float proximity_read_left_front_17 = 0;
static float proximity_read_right_front_17 =0;

uint8_t has_turned = 1; //variable permettant de ne pas prendre plusieurs photos du même mur
static uint8_t wall_on_right = 0 ;
static uint8_t free_to_turn = 0 ;

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){

//mesure des capteurs de proximité
		proximity_read_left = 0;
		proximity_read_right = 0;
		proximity_read_left_front_49 = 0;
		proximity_read_right_front_49 = 0;
		proximity_read_left_front_17 = 0;
		proximity_read_right_front_17 =0;
		proximity_measure(5);

		uint8_t red_val = RGB_MAX_INTENSITY/10;
		uint8_t green_val = RGB_MAX_INTENSITY;
		uint8_t	blue_val = RGB_MAX_INTENSITY/10;

		time = chVTGetSystemTime();

/*MACHINE D'ETATS
 * Le robot suit la règle de la main droite, i.e il tourne à droite à chaque fois qu'il peut  */

		switch(current_state){

			case WALL_IN_FRONT:
				if(is_path_open(proximity_read_right)){
					position_update(0,0);
					rotation_update(1);
					motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
					free_to_turn = 0;

				}else if(is_path_open(proximity_read_left)) {
					position_update(0,0);
					rotation_update(-1);
					motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, TURN_SPEED, -TURN_SPEED);
					current_state = TURNING;
					free_to_turn = 0;
				}else{
					position_update(0,0);
					rotation_update(2);
					motors_set_position(NB_STEP_U_TURN, NB_STEP_U_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
					free_to_turn = 0;
				}
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
					motors_set_position(NB_STEP_OPENING, NB_STEP_OPENING, NORMAL_SPEED, NORMAL_SPEED);
					current_state = JUNCTION ;
					free_to_turn = 0;
					wall_on_right = 0;

				}else{
					float correction = trajectory_correction(proximity_read_right, proximity_read_right_front_49, proximity_read_left, proximity_read_left_front_49);
					right_motor_set_speed(NORMAL_SPEED - correction);
					left_motor_set_speed(NORMAL_SPEED + correction);
				}
				if(!free_to_turn) {
					if(!is_path_open(proximity_read_right))	{
						set_rgb_led(3,red_val, green_val, blue_val);
						wall_on_right = 1;
					}
					if(wall_on_right && is_path_open(proximity_read_right)) {
						set_rgb_led(2,red_val, green_val, blue_val);
						free_to_turn = 1;
						wall_on_right = 0;
					}
				}
				break;

			case TURNING:
				motors_update_target_reached();
				if(motors_get_reached() == 1) {
					left_motor_set_speed(NORMAL_SPEED);
					right_motor_set_speed(NORMAL_SPEED);
					has_turned = 1 ;
					nb_step_start_line_right = 0;
					nb_step_start_line_left = 0;
					current_state = MOVING;
				}
				if(abs(current_xpos) < 100 && abs(current_ypos) < 100) {
					current_state = FINISH;
				}
				break;

			case JUNCTION:
				motors_update_target_reached();
				if(motors_get_reached() == 1) {
					position_update(step_before_turn_right,step_before_turn_left);
					rotation_update(1);
					motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
				}
				break;


			case STARTING:
	    		chSequentialStreamRead(&SD3, (uint8_t*) &cmd, 1);
	    		chprintf((BaseSequentialStream *)&SD3, "Cmd = %i \n", cmd);
	    		if(cmd == ASCII_VALUE_OF_1) {
	    			switch_state(MOVING);
	    			playMelody(7, 0, 0);
	    		}
				break;

			case FINISH:

				break;
		}
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

// afin d'avoir des mesures plus stables, il est possible de les moyenner sur le nombre souhaité
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
	error = ((0.5*read_left + 2*(read_left_49+LEFT_PROX_OFFSET)) - (0.5*read_right + 2*read_right_49));
	return KP*error;
}

/*rotation takes -1,0,1,2 for values modulo could have been better but not sure if it handles negative values
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
//	chprintf((BaseSequentialStream *)&SD3, "delta = %i \n", delta_steps);
//	chprintf((BaseSequentialStream *)&SD3, "step start = %i \n", nb_step_start_line_right);
//	chprintf((BaseSequentialStream *)&SD3, "step end = %i \n", right_motor_get_pos());
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
	chprintf((BaseSequentialStream *)&SD3, "X = %i \n", current_xpos);
	chprintf((BaseSequentialStream *)&SD3, "Y = %i \n", current_ypos);
}

uint8_t get_state() {
	return current_state;
}

void switch_state(uint8_t new_state) {
	current_state = new_state;
}

/* Demarrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO, Navigation, NULL);
}



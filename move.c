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
#include <audio/play_melody.h>

//#define NB_PROX_SENSORS 8 a taej si sa marche sans
#define LEFT_IR_SENSOR 5
#define RIGHT_IR_SENSOR 2
#define RIGHT_FRONT_17_IR_SENSOR 0
#define LEFT_FRONT_17_IR_SENSOR 7
#define LEFT_FRONT_49_IR_SENSOR 6
#define RIGHT_FRONT_49_IR_SENSOR 1

#define PROXIMITY_THRESHOLD_SIDES 150 // ??cm
#define PROXIMITY_THRESHOLD_FRONT 400
#define PROXIMITY_OPEN_THRESHOLD 100

#define NB_STEP_QUARTER_TURN   320
#define NB_STEP_U_TURN   650
static uint8_t current_state = MOVING ;
static uint8_t free_to_turn = 1 ;

#define SMALL_CORRECTION 0.7
#define MEDIUM_CORRECTION 0.5
#define BIG_CORRECTION 0.3
/**/

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){
		// Pour initialisation plus fiable
//		chThdSleepMilliseconds(300);
		float proximity_read_left = get_prox(LEFT_IR_SENSOR);
		float proximity_read_right = get_prox(RIGHT_IR_SENSOR);
		float proximity_read_left_front_49 = get_prox(LEFT_FRONT_49_IR_SENSOR);
		float proximity_read_right_front_49 = get_prox(RIGHT_FRONT_49_IR_SENSOR);
		float proximity_read_left_front_17 = get_prox(LEFT_FRONT_17_IR_SENSOR);
		float proximity_read_right_front_17 = get_prox(RIGHT_FRONT_17_IR_SENSOR);
//		chprintf((BaseSequentialStream *)&SD3, "Right = %lf \n", proximity_read_right_front_17);
//		chprintf((BaseSequentialStream *)&SD3, "Left = %lf \n", proximity_read_left_front_17);
//		uint8_t red_val = RGB_MAX_INTENSITY/10;
//		uint8_t green_val = RGB_MAX_INTENSITY/10;
//		uint8_t	blue_val = RGB_MAX_INTENSITY;

//		if(proximity_read_left > PROXIMITY_THRESHOLD_FRONT) {
//			set_rgb_led(2,red_val, green_val, blue_val);
//		}else{
//			set_rgb_led(2, 0, 0, 0);
//		}
//		if(proximity_read_right > PROXIMITY_THRESHOLD_FRONT) {
//			set_rgb_led(1,red_val, green_val, blue_val);
//		}else{
//			set_rgb_led(1, 0, 0, 0);
//		}
//		if(proximity_read_left_front_49 > PROXIMITY_THRESHOLD_FRONT) {
//			set_led(3, 2);
//		}else{
//			set_led(3, 0);
//		}
//		if(proximity_read_right_front_49 > PROXIMITY_THRESHOLD_FRONT) {
//			set_led(1, 2);
//		}else{
//			set_led(1, 0);
//		}
//		if(proximity_read_left_front_17 > PROXIMITY_THRESHOLD_FRONT) {
//			set_rgb_led(3,red_val, green_val, blue_val);
//		}else{
//			set_rgb_led(3, 0, 0, 0);
//		}
//		if(proximity_read_right_front_17 > PROXIMITY_THRESHOLD_FRONT) {
//			set_rgb_led(0,red_val, green_val, blue_val);
//		}else{
//			set_rgb_led(0, 0, 0, 0);
//		}



		switch(current_state){

			case WAITING:
				if(is_path_open(proximity_read_right)){
					motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
					chThdYield();
					free_to_turn = 0;
				}else if(is_path_open(proximity_read_left)) {
					motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, TURN_SPEED, -TURN_SPEED);
					current_state = TURNING;
					chThdYield();
					free_to_turn = 0;
				}else{
					motors_set_position(NB_STEP_U_TURN, NB_STEP_U_TURN, -TURN_SPEED, TURN_SPEED);
					current_state = TURNING;
					chThdYield();
				}
				break;

			case MOVING:
				if(is_wall(proximity_read_left_front_17) && is_wall(proximity_read_right_front_17)) {

					motors_stop();
					current_state = WAITING ;

				}else if(is_path_open(proximity_read_right) && free_to_turn){

				current_state = WAITING ;
				chThdSleepMilliseconds(400);

				}else{

					left_motor_set_speed(trajectory_correction(proximity_read_right_front_49)*NORMAL_SPEED);
					right_motor_set_speed(trajectory_correction(proximity_read_left_front_49)*NORMAL_SPEED);

				}
				if(!free_to_turn && !is_path_open(proximity_read_right)) {
								free_to_turn = 1;
				}
				break;

			case TURNING:
				motors_update_target_reached();
				if(motors_get_reached() == 1) {
					left_motor_set_speed(NORMAL_SPEED);
					right_motor_set_speed(NORMAL_SPEED);
					current_state = MOVING;
				}
				break;

		}

	}
}

//uint8_t is_wall_in_front(float left_read, float right_read) {
//	return ((left_read > PROXIMITY_THRESHOLD_FRONT) && (right_read > PROXIMITY_THRESHOLD_FRONT));
//}

uint8_t is_wall(float read) {
	return (read > PROXIMITY_THRESHOLD_FRONT);
}

uint8_t is_path_open(float read) {
	return (read < PROXIMITY_OPEN_THRESHOLD);
}


float trajectory_correction(float read) {

	if(read > 300 && read > 400){
		return SMALL_CORRECTION;
	}
	if(read > 400 && read < 500){
		return MEDIUM_CORRECTION;
	}
	if(read > 500){
		return BIG_CORRECTION;
	}
	return 1.0;
}


/* Demarrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO, Navigation, NULL);
}



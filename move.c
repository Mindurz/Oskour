#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <sensors/proximity.h>
#include <motors.h>
//#include <sensors/VL53L0X/VL53L0X.h>
//#include <leds.h>
//#include <audio/play_melody.h>
//#include <spi_comm.h>
//#include <audio_processing.h>
#include "Move.h"
#include <main.h>
//#include <process_image.h>

#define VITESSE_NORMALE 600
#define LEGERE_CORRECTION 30
#define NB_PROX_SENSORS 8
#define SAFETY_PROX_VALUE 200
#define FRONT_RIGHT_IR_SENSOR 0
#define DIAGONALE_RIGHT_IR_SENSOR 1
#define RIGHT_IR_SENSOR 2
#define LEFT_IR_SENSOR 5
#define DIAGONALE_LEFT_IR_SENSOR 6
#define FRONT_LEFT_IR_SENSOR 7
#define QUARTER_TURN 90
#define U_TURN 180
#define MIN_TIME_OF_FLIGHT_DISTANCE 50
#define MIN_IR_VALUE 70
#define THRESHOLD_IR_DETECTS_WALL 100
#define NORMAL_SETTING 0,0
#define MAX_FINISH_TOF_DIST 180
#define MIN_FINISH_TOF_DIST 120

/**/

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){
			// Pour initialisation plus fiable
			chThdSleepMilliseconds(300);
			float proximity_read_left = get_prox(LEFT_IR_SENSOR);
			float proximity_read_right = get_prox(RIGHT_IR_SENSOR);
			float proximity_read_left_front_49 = get_prox(LEFT_FRONT_49_IR_SENSOR);
			float proximity_read_right_front_49 = get_prox(RIGHT_FRONT_49_IR_SENSOR);
			float proximity_read_left_front_17 = get_prox(LEFT_FRONT_17_IR_SENSOR);
			float proximity_read_right_front_17 = get_prox(RIGHT_FRONT_17_IR_SENSOR);
	//		chprintf((BaseSequentialStream *)&SDU1, "Left = %lf \n", proximity_read_left);
	//		chprintf((BaseSequentialStream *)&SDU1, "Right = %lf \n", proximity_read_right);
			uint8_t red_val = RGB_MAX_INTENSITY/10;
			uint8_t green_val = RGB_MAX_INTENSITY/10;
			uint8_t	blue_val = RGB_MAX_INTENSITY;

			if(proximity_read_left > PROXIMITY_THRESHOLD_FRONT) {
				set_rgb_led(2,red_val, green_val, blue_val);
			}else{
				set_rgb_led(2, 0, 0, 0);
			}
			if(proximity_read_right > PROXIMITY_THRESHOLD_FRONT) {
				set_rgb_led(1,red_val, green_val, blue_val);
			}else{
				set_rgb_led(1, 0, 0, 0);
			}
			if(proximity_read_left_front_49 > PROXIMITY_THRESHOLD_FRONT) {
				set_led(3, 2);
			}else{
				set_led(3, 0);
			}
			if(proximity_read_right_front_49 > PROXIMITY_THRESHOLD_FRONT) {
				set_led(1, 2);
			}else{
				set_led(1, 0);
			}
			if(proximity_read_left_front_17 > PROXIMITY_THRESHOLD_FRONT) {
				set_rgb_led(3,red_val, green_val, blue_val);
			}else{
				set_rgb_led(3, 0, 0, 0);
			}
			if(proximity_read_right_front_17 > PROXIMITY_THRESHOLD_FRONT) {
				set_rgb_led(0,red_val, green_val, blue_val);
			}else{
				set_rgb_led(0, 0, 0, 0);
		}

		switch(current_state){

			case WAITING:
				if(is_wall_on_right(proximity_read_right)){
					if(is_wall_on_left(proximity_read_left)){
						motors_set_position(2*NB_STEP_QUARTER_TURN, 2*NB_STEP_QUARTER_TURN, -NORMAL_SPEED, NORMAL_SPEED);
						current_state = TURNING;
					}else{
						motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, NORMAL_SPEED, -NORMAL_SPEED);
						current_state = TURNING;
					}
				}else if(is_wall_on_left(proximity_read_left)) {
						motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -NORMAL_SPEED, NORMAL_SPEED);
						current_state = TURNING;
					}
				chprintf((BaseSequentialStream *)&SD3, "Niktoi \n");
				break;

			case MOVING:
				if(is_wall_in_front(proximity_read_left_front_17, proximity_read_right_front_17) && (current_state == MOVING)) {
					motors_stop();
					current_state = WAITING ;
				}else{
					left_motor_set_speed(NORMAL_SPEED);
					right_motor_set_speed(NORMAL_SPEED);
				}
				chprintf((BaseSequentialStream *)&SD3, "Pute \n");
				break;

			case TURNING:
				if(motors_get_reached()) {
					left_motor_set_speed(NORMAL_SPEED);
					right_motor_set_speed(NORMAL_SPEED);
					current_state = MOVING;
				}
				chprintf((BaseSequentialStream *)&SD3, "Suce \n");
				break;

		}
	}
}

/* Démarrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+1, Navigation, NULL);
}



#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <sensors/proximity.h>
#include <motors.h>
#include "leds.h"
#include "move.h"
#include <main.h>


#define NB_PROX_SENSORS 8
#define LEFT_IR_SENSOR 5
#define RIGHT_IR_SENSOR 2
#define RIGHT_FRONT_17_IR_SENSOR 0
#define LEFT_FRONT_17_IR_SENSOR 7
#define LEFT_FRONT_49_IR_SENSOR 6
#define RIGHT_FRONT_49_IR_SENSOR 1
#define PROXIMITY_THRESHOLD_SIDES 150
#define PROXIMITY_THRESHOLD_FRONT 100

/**/

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

//	systime_t time;

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
	}
}

/* Demarrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO, Navigation, NULL);
}



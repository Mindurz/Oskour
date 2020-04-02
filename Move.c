#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <sensors/proximity.h>
#include <motors.h>

#include "Move.h"
#include <main.h>

#define NB_PROX_SENSORS 8
#define LEFT_IR_SENSOR 5
#define QUARTER_TURN 90
#define U_TURN 180


/**/

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//systime_t time;

	// Pour initialisation plus fiable
	chThdSleepMilliseconds(300);
	float proximity_read = get_prox(LEFT_IR_SENSOR);
	chprintf((BaseSequentialStream *)&SDU1, "valeur = %lf \n", proximity_read);
}

/* D�marrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+1, Navigation, NULL);
}



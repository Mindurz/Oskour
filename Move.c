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

//	uint8_t activation = ARRET, mode = ARRET;

	// Pour initialisation plus fiable
	chThdSleepMilliseconds(300);
	float proximity_read = get_prox(LEFT_IR_SENSOR);
	chprintf((BaseSequentialStream *)&SDU1, "valeur = %lf \n", proximity_read);
}

/* D�marrage du thread */

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+1, Navigation, NULL);
}



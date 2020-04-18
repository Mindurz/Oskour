#include "start.h"

#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>

#include <main.h>
#include <move.h>

static uint8_t started = 0;
static uint8_t cmd = 0;

static THD_WORKING_AREA(waStart, 128);
static THD_FUNCTION(Start, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     systime_t time;

     while (1) {
    	 time = chVTGetSystemTime();
    	 // default command is FRONT
    	 cmd = 0 ;
    	 /* chSequentialStreamRead seems to disable the thread while waiting for
    	  * for an input in the serial buffer */
    	while(1){
    		if(!started) {

    			chSequentialStreamRead(&SD3, (uint8_t*) &cmd, 1);
    			chprintf((BaseSequentialStream *)&SD3, "Cmd = %i \n", cmd);

    			if(cmd == 49) {
    				switch_state(MOVING);
    				started = 1;
    			}
    		}
    		chThdSleepUntilWindowed(time, time + MS2ST(10));
    	 }
     }
}

void start_start(void){
	chThdCreateStatic(waStart, sizeof(waStart), NORMALPRIO, Start, NULL);
}

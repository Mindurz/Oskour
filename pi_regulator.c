#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#define ARW_THRESHOLD 15

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    static float error = 0, integral = 0, decalage_speed = 0;
    static const float Kp =800 , Ki = 3.5, Kp_decalage = 2;


    while(1){
        time = chVTGetSystemTime();

        error = get_distance_cm() - 10.0;


        integral += error;
        if(integral > ARW_THRESHOLD){
        	integral = ARW_THRESHOLD;
        } else if (integral < -ARW_THRESHOLD) {
        	integral = -ARW_THRESHOLD;
        }
        speed = Kp*error + Ki*integral;

        //decalage_speed = Kp_decalage*get_decalage_pxl();

        //applies the speed from the PI regulator
//		right_motor_set_speed(speed-decalage_speed);
//		left_motor_set_speed(speed+decalage_speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

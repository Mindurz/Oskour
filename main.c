#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include <chprintf.h>
#include "hal.h"

#include "memory_protection.h"
#include "sensors/proximity.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include <communication.h>
#include "spi_comm.h"

#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <leds.h>
#include <camera/po8030.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>

//#include <pi_regulator.h>
#include <process_image.h>
#include <motors_control.h>
#include <move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();

    dac_start();

    // starts the time of flight sensor
    VL53L0X_start();

    //starts the camera
    dcmi_start();
    po8030_start();

    // for rgb led use
    spi_comm_start();

	//inits the motors
	motors_init();
	proximity_start();

	//stars the threads for the pi regulator and the processing of the image
//	pi_regulator_start();
	process_image_start();
	navigation_start();

//	playMelodyStart();

    /* Infinite loop. */
    while (1) {

//    	waits 1 second
    	chThdSleepMilliseconds(1000);
//    	distance = VL53L0X_get_dist_mm();
//    	chprintf((BaseSequentialStream *)&SD3, "dist = %lf \n", distance);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

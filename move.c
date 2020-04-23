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
#define PROXIMITY_THRESHOLD_FRONT 220
#define PROXIMITY_OPEN_THRESHOLD 80

#define NB_STEP_QUARTER_TURN   320
#define NB_STEP_U_TURN   650
#define NB_STEP_OPENING 150

#define LEFT_PROX_OFFSET 20
#define KP 0.02

#define ASCII_VALUE_OF_1 49

static uint8_t current_state = STARTING ;
static uint8_t wall_on_right = 0 ;
static uint8_t free_to_turn = 0 ;
static int8_t current_direction = X_POS;
static float current_xpos = 0;
static float current_ypos = 0;
static float proximity_read_left = 0;
static float proximity_read_right = 0;
static float proximity_read_left_front_49 = 0;
static float proximity_read_right_front_49 = 0;
static float proximity_read_left_front_17 = 0;
static float proximity_read_right_front_17 =0;

/**/

uint8_t has_turned = 1;
uint8_t counter = 0;

static THD_WORKING_AREA(waNavigation, 1024);
static THD_FUNCTION(Navigation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        // Pour initialisation plus fiable
//        chThdSleepMilliseconds(300);
        proximity_read_left = 0;
        proximity_read_right = 0;
        proximity_read_left_front_49 = 0;
        proximity_read_right_front_49 = 0;
        proximity_read_left_front_17 = 0;
        proximity_read_right_front_17 =0;
        proximity_measure(5);
//        chprintf((BaseSequentialStream *)&SD3, "Right = %lf \n", proximity_read_right_front_17);
//        chprintf((BaseSequentialStream *)&SD3, "Left = %lf \n", proximity_read_left_front_17);
        uint8_t red_val = RGB_MAX_INTENSITY/10;
        uint8_t green_val = RGB_MAX_INTENSITY;
        uint8_t    blue_val = RGB_MAX_INTENSITY/10;

        time = chVTGetSystemTime();

//        chprintf((BaseSequentialStream *)&SD3, "nb_line = %i, counter = %i \n", get_nbLine(), counter);
//        ++counter;
//        chThdSleepMilliseconds(5000);
//        chprintf((BaseSequentialStream *)&SD3, "Y = %lf \n", current_ypos);
        switch(current_state){

            case WALL_IN_FRONT:
                if(is_path_open(proximity_read_right)){
                    motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
                    current_state = TURNING;
                    rotation_update(1);
                    chThdYield();
                    free_to_turn = 0;

                }else if(is_path_open(proximity_read_left)) {
                    motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, TURN_SPEED, -TURN_SPEED);
                    current_state = TURNING;
                    rotation_update(1);
                    chThdYield();
                    free_to_turn = 0;
                }else{
                    motors_set_position(NB_STEP_U_TURN, NB_STEP_U_TURN, -TURN_SPEED, TURN_SPEED);
                    current_state = TURNING;
                    rotation_update(2);
                    chThdYield();
                    free_to_turn = 0;
                }
                break;

            case MOVING:
                if(is_wall(proximity_read_left_front_17) && is_wall(proximity_read_right_front_17)) {
                    motors_stop();
                    current_state = WALL_IN_FRONT ;

                }else if(is_path_open(proximity_read_right) && free_to_turn){

                    motors_set_position(NB_STEP_OPENING, NB_STEP_OPENING, NORMAL_SPEED, NORMAL_SPEED);
                    current_state = JUNCTION ;
                    position_update();
                    free_to_turn = 0;
                    wall_on_right = 0;

                }else{

                    float correction = trajectory_correction(proximity_read_right, proximity_read_right_front_49, proximity_read_left, proximity_read_left_front_49);
                    right_motor_set_speed(NORMAL_SPEED - correction);
                    left_motor_set_speed(NORMAL_SPEED + correction);
                    position_update();

                }
                if(!free_to_turn) {
                    if(!is_path_open(proximity_read_right))    {
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
                    set_rgb_led(3, 0, 0, 0);
                    set_rgb_led(2, 0, 0, 0);
                    current_state = MOVING;
                }
                break;

            case JUNCTION:
                motors_update_target_reached();
                if(motors_get_reached() == 1) {
                    motors_set_position(NB_STEP_QUARTER_TURN, NB_STEP_QUARTER_TURN, -TURN_SPEED, TURN_SPEED);
                    current_state = TURNING;
                }
                break;


            case STARTING:
                if(!started) {
                    chSequentialStreamRead(&SD3, (uint8_t*) &cmd, 1);
                    chprintf((BaseSequentialStream *)&SD3, "Cmd = %i \n", cmd);
                    if(cmd == ASCII_VALUE_OF_1) {
                        switch_state(MOVING);
                        started = 1;
                    }
                }
                break;
        }
//        chprintf((BaseSequentialStream *)&SD3, "prox right = %lf \n", proximity_read_right);
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

//uint8_t is_wall_in_front(float left_read, float right_read) {
//    return ((left_read > PROXIMITY_THRESHOLD_FRONT) && (right_read > PROXIMITY_THRESHOLD_FRONT));
//}

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



#include <stdlib.h>
#include <motors_control.h>
#include <motors.h>

static uint16_t left_step_to_target = 0;
static uint16_t right_step_to_target = 0;

static uint8_t target_right_reached = TARGET_REACHED;
static uint8_t target_left_reached = TARGET_REACHED;

void motors_update_target_reached(void){

	int left_motor_pos = 0;
	int right_motor_pos = 0;

	left_motor_pos = left_motor_get_pos();
	right_motor_pos = right_motor_get_pos();

	if(abs(left_motor_pos) >= right_step_to_target && target_left_reached == TARGET_NOT_REACHED){
		target_left_reached = TARGET_REACHED;
	}
	if(abs(right_motor_pos) >= left_step_to_target && target_right_reached == TARGET_NOT_REACHED ){
		target_right_reached = TARGET_REACHED;
	}
}

void motors_set_target(int32_t target_right, int32_t target_left, int16_t speed_right, int16_t speed_left){

    target_right_reached = TARGET_NOT_REACHED;
    target_left_reached = TARGET_NOT_REACHED;



    left_motor_set_pos(0);
    right_motor_set_pos(0);

    left_step_to_target = target_left;
    right_step_to_target = target_right;

	left_motor_set_speed(speed_left);
	right_motor_set_speed(speed_right);

}


uint8_t motors_is_target_reached(void){
	uint8_t reached = TARGET_NOT_REACHED;
	if(target_right_reached == target_left_reached){
		reached = target_right_reached;
	}
	return reached;
}

void motors_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}




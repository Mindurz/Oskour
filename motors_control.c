
#include <motors_control.h>
#include <motors.h>
#include <math.h>

static uint16_t left_counter_target = 0;
static uint16_t right_counter_target = 0;

static uint8_t position_right_reached = POSITION_REACHED;
static uint8_t position_left_reached = POSITION_REACHED;


void motors_update_target_reached(void){

	int left_motor_pos = 0;
	int right_motor_pos = 0;

	left_motor_pos = left_motor_get_pos();
	right_motor_pos = right_motor_get_pos();

	if(abs(left_motor_pos) >= right_counter_target &&
	   position_left_reached == POSITION_NOT_REACHED){
		position_left_reached = POSITION_REACHED;
	}
	if(abs(right_motor_pos) >= left_counter_target &&
	   position_right_reached == POSITION_NOT_REACHED ){
		position_right_reached = POSITION_REACHED;
	}
}

void motors_set_position(int32_t position_r, int32_t position_l, int16_t speed_r, int16_t speed_l){

    position_right_reached = POSITION_NOT_REACHED;
    position_left_reached = POSITION_NOT_REACHED;

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    left_counter_target = position_l;
    right_counter_target = position_r;

	left_motor_set_speed(speed_l);
	right_motor_set_speed(speed_r);
}


uint8_t motors_get_reached(void){
	uint8_t reached = POSITION_NOT_REACHED;
	if(position_right_reached == position_left_reached){
		reached = position_right_reached;
	}
	return reached;
}

void motors_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}



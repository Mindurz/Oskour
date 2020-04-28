#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_BUFFER_SIZE		640
#define INTENSITE_MAX			100
#define NB_SAMPLES_OFFSET		200
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			50

void find_line(uint8_t *buffer);
int get_decalage_pxl(void);
void process_image_start(void);
uint8_t get_nbLine(void);

#endif /* PROCESS_IMAGE_H */

//left_motor_set_speed(LINEAR_SPEED - (2*get_prox_values()->delta[0] + get_prox_values()->delta[1])*LINEAR_SPEED/(IR_VAL_MAX*3));
//right_motor_set_speed(LINEAR_SPEED - (2*get_prox_values()->delta[7] + get_prox_values()->delta[6])*LINEAR_SPEED/(IR_VAL_MAX*3));

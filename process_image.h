#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_BUFFER_SIZE		640
#define INTENSITE_MAX			100
#define NB_SAMPLES_OFFSET		200
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			50

uint8_t extract_nb_line(uint8_t *buffer);
void process_image_start(void);
uint8_t get_nb_line(void);
void set_nb_line(uint8_t setter);

#endif /* PROCESS_IMAGE_H */

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_BUFFER_SIZE		640
#define INTENSITE_MAX			100
#define NB_SAMPLES_OFFSET		200
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			50

float get_distance_cm(void);
void find_line(uint8_t *buffer);
int get_decalage_pxl(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */

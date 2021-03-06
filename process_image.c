#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <move.h>
#include <camera/po8030.h>
#include "leds.h"
#include <sensors/VL53L0X/VL53L0X.h>

#include <process_image.h>

static float distance_cm = 0;
static int decalage_pxl = 0;

static uint16_t line_position = IMAGE_BUFFER_SIZE/2;    //middle
static uint8_t nbLine = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint8_t extract_line_width(uint8_t *buffer){

    uint8_t nb_line = 0;

    uint16_t i = 0, begin = 0, end = 0, width = 0;
    uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
    uint32_t mean = 0;
//    uint8_t red_val = RGB_MAX_INTENSITY/10;
//    uint8_t green_val = RGB_MAX_INTENSITY/10;
//    uint8_t    blue_val = RGB_MAX_INTENSITY;

//    static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

    //performs an average
    for(uint16_t j = 0 ; j < IMAGE_BUFFER_SIZE ; j++){
        mean += buffer[j];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do{
        wrong_line = 0;
        //search for a begin
        while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
        {
            //the slope must at least be WIDTH_SLOPE wide and is compared
            //to the mean of the image
            if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
            {
                begin = i;
                stop = 1;
            }
            i++;
        }
        //if a begin was found, search for an end
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin) {
            stop = 0;
            while(stop == 0 && i < IMAGE_BUFFER_SIZE) {
                if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean) {
                    end = i;
                    stop = 1;
                }
                i++;
            }
            //if an end was not found
            if (i > IMAGE_BUFFER_SIZE || !end) {
                line_not_found = 1;
            }
        } else {//if no begin was found
             line_not_found = 1;
        }
        //if a line has been detected, continues the search
        if(!line_not_found && ((end-begin) > MIN_LINE_WIDTH)) {
            nb_line++;
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;


        }

//        if(end && begin) {
//            nb_line++;
//        }

    }while(wrong_line);

    return nb_line;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    while(1){
        uint16_t distance_to_wall = VL53L0X_get_dist_mm();
        if(distance_to_wall < 135 && get_has_turned() && (get_state() == MOVING)){
            //starts a capture
            set_rgb_led(2,0, 0, 0);
            dcmi_capture_start();
            //waits for the capture to be done
            wait_image_ready();
            //signals an image has been captured
            chBSemSignal(&image_ready_sem);
            set_has_turned(0);
        }
//        chThdSleepMilliseconds(500);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;

    uint8_t *img_buff_ptr;
    uint8_t image[IMAGE_BUFFER_SIZE] = {0};
//    uint8_t nbLine = 0;
	static uint8_t unefoissurdeux = 0;

    bool send_to_computer = false;

    while(1){
        //waits until an image has been captured
        chBSemWait(&image_ready_sem);
        //gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

        //Extracts only the red pixels
        for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
            //extracts first 5bits of the first byte
            //takes nothing from the second byte
            image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
        }

        //search for a line in the image and gets its width in pixels
        nbLine = extract_line_width(image);
        chprintf((BaseSequentialStream *)&SD3, "nbLine = %i \n", nbLine);


        if(send_to_computer){
            //sends to the computer the image
            SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
        }
        //invert the bool
//        send_to_computer = !send_to_computer;
    }
}

uint16_t get_line_position(void){
    return line_position;
}

void process_image_start(void){
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint8_t get_nbLine() {
    return nbLine;
}

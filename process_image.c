#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static float distance_cm = 0;
static int decalage_pxl = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static uint16_t code = 0;    //middle

void find_line(uint8_t *buffer) {

    uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
    uint16_t i = 0, begin = 0, end = 0, nb_line = 0;
    uint32_t mean = 0;

    //performs an average

    for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
   	 mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do
    {
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
   	 if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
   	 {
   	 	stop = 0;

   	 	while(stop == 0 && i < IMAGE_BUFFER_SIZE)
   	 	{
   	     	if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
   	     	{
   	         	end = i;
   	         	stop = 1;
   	     	}
   	     	i++;
   	 	}
   	 	//if an end was not found
   	 	if (i > IMAGE_BUFFER_SIZE || !end)
   	 	{
   	     	line_not_found = 1;
   	 	}
   	 }
   	 else//if no begin was found
   	 {
   	 	line_not_found = 1;
   	 }

   	 //if a line too small has been detected, continues the search
   	 if(!line_not_found)
     {
        if ((end-begin)>MIN_LINE_WIDTH)
        {
          nb_line++;
        }
   		  i = end;
   		  begin = 0;
   		  end = 0;
   		  stop = 0;
   		  wrong_line = 1;
   	 }
     if(end && begin){
       nb_line++;
     }
    }while(wrong_line);

    code = nb_line;
    chprintf((BaseSequentialStream *)&SDU1, "nb lignes %lf \n", nb_line);
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 0 + 1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 478, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

//	systime_t time;
//	systime_t time2;

    while(1){
//    	time = chVTGetSystemTime();

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

//		time2 = chVTGetSystemTime() - time;
//		chprintf((BaseSequentialStream *)&SDU1, "time = %d \n", time2);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	systime_t time;

//	uint8_t activation = ARRET, mode = ARRET;

	// Pour initialisation plus fiable

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t unefoissurdeux = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		if(unefoissurdeux != 0){
			unefoissurdeux--;
		} else {
			find_line(img_buff_ptr) ;
//			uint16_t moyenne = 0;
//			for(int i = 0; i< IMAGE_BUFFER_SIZE; i++){
//				image[i] = ((img_buff_ptr[2*i]  & 0b111) << 3);
//				image[i] += (img_buff_ptr[2*i+1] >> 5);
//				moyenne += image[i];
//			}
//			moyenne /= IMAGE_BUFFER_SIZE;
//
//			uint16_t debut_ligne = 0, fin_ligne = 0;
//			for(int i = 0; i < IMAGE_BUFFER_SIZE-5 ; i++){
//				if((image[i] > moyenne) && (image[i + 5] < moyenne)){
//					if((image[i] - image[i+5]) > 5){
//						debut_ligne = i+5;
//					}
//				} else if ((image[i] < moyenne) && (image[i + 5] > moyenne)){
//					if((image[i+5] - image[i])  > 5){
//						fin_ligne = i+5;
//					}
//				}
//			}
//			decalage_pxl = (debut_ligne+fin_ligne- IMAGE_BUFFER_SIZE)/2; // mean value centered in IMAGE_BUFFER_SIZE/2
//			distance_cm = - 0.142*(fin_ligne-debut_ligne)+ 29.94;
			//chprintf((BaseSequentialStream *)&SDU1, "Pixels = %d \n", fin_ligne-debut_ligne);
			//chprintf((BaseSequentialStream *)&SDU1, "Dist = %lf \n", distance_cm);
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
			unefoissurdeux+=2;
		}
    }
}


float get_distance_cm(void){
	return distance_cm;
}

int get_decalage_pxl(void){
	return decalage_pxl;
}
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

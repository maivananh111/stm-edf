/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern "C" void HAL_driver_init(void);
/**
 * Main application.
 */
void app_main(void){
	HAL_driver_init();

	while(1){
		printf("Hello world!\r\n");
		vTaskDelay(1000);
	}
}

/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common/log_monitor.h"

#include "peripherals/sysinfo.h"
#include "peripherals/gpio.h"
#include "peripherals/uart.h"


#define UART_RXBUF_SIZE 10
#define RECEPTION_MODE UART_RECEPTION_UNTIL_COMPLETED

extern "C" void HAL_driver_init(void);


uart_config_t u1_conf = {
	.baudrate = 115200,
	.interruptpriority = 0,
	.txpin = (char *)"A9",
	.rxpin = (char *)"A10",
};


void task_uart(void *){
    status_t ret;
	uart1->uart_init(&u1_conf);
    uart1->uart_set_endchar('\n');

	while(1){
		uart1->uart_transmit(UART_MODE_NORMAL, (uint8_t *)"Hello from STM32", strlen((char *)"Hello from STM32"));


        ret = uart1->uart_receive(UART_MODE_NORMAL, RECEPTION_MODE, UART_RXBUF_SIZE);
        if(!status_is_pass(&ret)){
            char *rxdata;
            status_t ret = uart1->uart_get_buffer((uint8_t **)&rxdata);
            if(!status_is_pass(&ret)){
                LOG_ERROR("UART", "Can't get UART data.");
            }
            else{
                LOG_WARN("UART RECEIVED", "%s", rxdata);
            }
            if(rxdata != NULL) free(rxdata);
        }


		vTaskDelay(1000);
	}
}


void app_main(void){
	HAL_driver_init();

	xTaskCreate(task_uart, "task_uart", bytes_to_words(4096), NULL, 5, NULL);
	while(1){
		LOG_MEM("MEM", "Free heap: %lu", dev_get_free_heap_size());
		vTaskDelay(1000);
	}
}







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
#include "peripherals/dma.h"
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


void uart1_event_handler(uart_event_t event, void *){
	if(event == UART_EVENT_TRANSMIT_COMPLETE){
		gpio_toggle(GPIOC, 13);
	}

	if(event == UART_EVENT_RECEIVE_COMPLETE){
		char *rxdata;
		status_t ret = uart1->uart_get_buffer((uint8_t **)&rxdata);
		if(!status_is_pass(&ret)){
			LOG_ERROR("UART", "Can't get UART data.");
		}
		else{
			LOG_WARN("UART RECEIVED", "%s", rxdata);
		}
		if(rxdata != NULL) free(rxdata);

		uart1->uart_receive(UART_MODE_INTERRUPT, RECEPTION_MODE, UART_RXBUF_SIZE);
	}
}


void task_uart(void *){
	uart1->uart_init(&u1_conf);
	uart1->uart_add_event_handler(uart1_event_handler, NULL);
    uart1->uart_set_endchar('\n');
	uart1->uart_receive(UART_MODE_INTERRUPT, RECEPTION_MODE, UART_RXBUF_SIZE);

	while(1){
		uart1->uart_transmit(UART_MODE_INTERRUPT, (uint8_t *)"Hello from STM32", strlen((char *)"Hello from STM32"));
		vTaskDelay(1000);
	}
}


void app_main(void){
	HAL_driver_init();
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	xTaskCreate(task_uart, "task_uart", bytes_to_words(4096), NULL, 5, NULL);
	while(1){
		LOG_MEM("MEM", "Free heap: %lu", dev_get_free_heap_size());
		vTaskDelay(1000);
	}
}







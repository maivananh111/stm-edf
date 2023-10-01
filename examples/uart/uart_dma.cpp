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


extern "C" void HAL_driver_init(void);

uart_config_t u1_conf = {
	.baudrate = 115200,
	.interruptpriority = 0,
	.txpin = (char *)"A9",
	.rxpin = (char *)"A10",
};
dma_config_t u1_txdma_conf = {
	.stream = DMA2_Stream7,
	.channel = DMA_CHANNEL4,
	.direction = DMA_MEM_TO_PERIPH,
	.mode = DMA_MODE_NORMAL,
	.interruptpriority = 5,
};
dma_config_t u1_rxdma_conf = {
	.stream = DMA2_Stream5,
	.channel = DMA_CHANNEL4,
	.direction = DMA_PERIPH_TO_MEM,
	.mode = DMA_MODE_NORMAL,
	.interruptpriority = 6,
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
		uart1->uart_receive(UART_MODE_DMA, UART_RECEPTION_UNTIL_COMPLETED, 3);
	}
}

void dma2_str5_event_handler(dma_event_t event, void *){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		char *rxdata;
		status_t ret = uart1->uart_get_buffer((uint8_t **)&rxdata);
		if(!status_is_pass(&ret)){
			LOG_ERROR("UART", "Can't get UART data.");
		}
		else{
			LOG_WARN("UART RECEIVED", "%s", rxdata);
		}
		if(rxdata != NULL) free(rxdata);
		uart1->uart_abort_receive(UART_MODE_DMA);
		uart1->uart_receive(UART_MODE_DMA, UART_RECEPTION_UNTIL_COMPLETED, 3);
	}
}

void dma2_str7_event_handler(dma_event_t event, void *){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		uart1->uart_abort_transmit(UART_MODE_DMA);
	}
}

void task_uart(void *){
	dma2_stream7->dma_init(&u1_txdma_conf);
	dma2_stream7->dma_add_event_handler(dma2_str7_event_handler, NULL);
	dma2_stream5->dma_init(&u1_rxdma_conf);
	dma2_stream5->dma_add_event_handler(dma2_str5_event_handler, NULL);

	uart1->uart_init(&u1_conf);
	uart1->uart_add_dma(dma2_stream7, dma2_stream5);
	uart1->uart_add_event_handler(uart1_event_handler, NULL);

	uart1->uart_receive(UART_MODE_DMA, UART_RECEPTION_UNTIL_COMPLETED, 3);

	while(1){
		uart1->uart_transmit(UART_MODE_DMA, (uint8_t *)"Hello from STM32", strlen((char *)"Hello from STM32"));
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







/*
 * logconfig.cpp
 *
 *  Created on: Aug 22, 2023
 *      Author: anh
 */

#include "system/logconfig.h"
#include "common/log_monitor.h"

#include "drivers/gpio.h"
#include "drivers/rcc.h"



#if CONFIG_USE_LOG_MONITOR
#if CONFIG_LOG_MONITOR_OUTPUT_UART && CONFIG_PERIPH_UART_EN
static uart_t log_uart;
static uart_config_t uart_log_conf = {
	.baudrate = CONFIG_LOG_MONITOR_UART_BAUDRATE,
	.txpin = (char *)CONFIG_LOG_MONITOR_UART_TXPIN,
	.rxpin = (char *)CONFIG_LOG_MONITOR_UART_RXPIN,
};
static void uart_log_init(void);
#endif /* CONFIG_LOG_MONITOR_OUTPUT_UARTT */
static void log_out(char *log);


void logstream_init(void){
#if CONFIG_LOG_MONITOR_OUTPUT_UART
	uart_log_init();
	log_monitor_init(log_out);
#endif
#if CONFIG_LOG_MONITOR_OUTPUT_USBVCP
	MX_USB_DEVICE_Init();
	log_monitor_init(usb_log);
#endif
	log_out((char *)"\r\n\r\n");
	log_out((char *)"\r\n\r\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*Target starting*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*\r\n");
}


#if CONFIG_LOG_MONITOR_OUTPUT_UART
static void uart_log_init(void){
	log_uart = uart_str_decode((char *)CONFIG_LOG_MONITOR_UART_INSTANCE);
	log_uart->initialize(&uart_log_conf);
}



static void log_out(char *log){
#if CONFIG_LOG_MONITOR_OUTPUT_UART
	log_uart->transmit((uint8_t *)log, strlen(log), 1000);
#endif
#if CONFIG_LOG_MONITOR_OUTPUT_USBVCP
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t ret, in_it = xPortIsInsideInterrupt();
	(in_it == pdTRUE)? (ret = xSemaphoreTakeFromISR(log_semaph, &xHigherPriorityTaskWoken)) : (ret = xSemaphoreTake(log_semaph, 10));

	if(ret == pdTRUE){
		CDC_Transmit_FS((uint8_t *)log, strlen(log));
		(in_it == pdTRUE)? xSemaphoreGiveFromISR(log_semaph, &xHigherPriorityTaskWoken) : xSemaphoreGive(log_semaph);
	}
#endif
}
#endif /* CONFIG_LOG_MONITOR_OUTPUT_UART */

extern "C" int _write(int file, char *ptr, int len){
#if CONFIG_LOG_MONITOR_OUTPUT_UART
	log_uart->transmit((uint8_t *)ptr, len, 1000);
#endif
#if CONFIG_LOG_MONITOR_OUTPUT_USBVCP
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t ret, in_it = xPortIsInsideInterrupt();
	(in_it == pdTRUE)? (ret = xSemaphoreTakeFromISR(log_semaph, &xHigherPriorityTaskWoken)) : (ret = xSemaphoreTake(log_semaph, 10));

	if(ret == pdTRUE){
		CDC_Transmit_FS((uint8_t *)ptr, len);
		(in_it == pdTRUE)? xSemaphoreGiveFromISR(log_semaph, &xHigherPriorityTaskWoken) : xSemaphoreGive(log_semaph);
	}
#endif
	fflush(stdout);

    return len;
}

#endif /* LOG_MONITOR */


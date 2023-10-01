/*
 * logconfig.h
 *
 *  Created on: Aug 22, 2023
 *      Author: anh
 */

#ifndef LOGCONFIG_INIT_H_
#define LOGCONFIG_INIT_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#if CONFIG_LOG_MONITOR_ENABLE
#if CONFIG_LOG_MONITOR_OUTPUT_USBVCP
#include "usb_device/usb_device.h"
#include "usb_device/usb_cdc/usbd_cdc_if.h"
#endif /* CONFIG_LOG_MONITOR_OUTPUT_USBVCP */
#if CONFIG_LOG_MONITOR_OUTPUT_UART
#include "drivers/uart.h"
#endif /* CONFIG_LOG_MONITOR_OUTPUT_UART */

#ifdef __cplusplus
extern "C"{
#endif


void logstream_init(void);

int _write(int file, char *ptr, int len);

#endif /* CONFIG_LOG_MONITOR_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* LOGCONFIG_INIT_H_ */

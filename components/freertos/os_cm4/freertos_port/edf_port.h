/*
 * edf_port.h
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */

#ifndef FREERTOS_EDF_PORT_H_
#define FREERTOS_EDF_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#if configUSE_IDLE_HOOK
void vApplicationIdleHook(void);
#endif
#if configUSE_TICK_HOOK
void vApplicationTickHook(void);
#endif
#if configUSE_MALLOC_FAILED_HOOK
void vApplicationMallocFailedHook(void);
#endif
#if configUSE_DAEMON_TASK_STARTUP_HOOK
void vApplicationDaemonTaskStartupHook(void);
#endif
#if configCHECK_FOR_STACK_OVERFLOW
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
#endif

#if configSUPPORT_STATIC_ALLOCATION
void vApplicationGetIdleTaskMemory  (StaticTask_t **ppxIdleTaskTCBBuffer,  StackType_t **ppxIdleTaskStackBuffer,  uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_EDF_PORT_H_ */

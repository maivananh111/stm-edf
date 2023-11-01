/*
 * edf_port.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: anh
 */

#include "freertos_port/edf_port.h"

#include "drivers/iwdg.h"
#include "drivers/system.h"
#include "drivers/sysinfo.h"
#if CONFIG_USE_LOG_MONITOR
#include "common/log_monitor.h"

static const char *TAG = (const char *)"RTOS_PORT";
#endif /* CONFIG_USE_LOG_MONITOR */

#if configUSE_IDLE_HOOK
void vApplicationIdleHook(void){
	/** Do something while cpu idle. */
#if CONFIG_WATCHDOG_ENABLE
	iwdg_refresh();
#endif /* CONFIG_RTOS_USE_IWDG */
	dev_cal_cpu_load_percent();
}
#endif

#if configUSE_TICK_HOOK
void vApplicationTickHook(void){

}
#endif

#if configUSE_DAEMON_TASK_STARTUP_HOOK
void vApplicationDaemonTaskStartupHook(void){

}
#endif

#if configCHECK_FOR_STACK_OVERFLOW
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName){
#if CONFIG_USE_LOG_MONITOR
	LOG_ERROR(TAG, "Stack overflow on %s.", pcTaskName);
#endif /* CONFIG_USE_LOG_MONITOR */
	for(uint32_t i=0; i< 80000000; i++) __NOP();
	__NVIC_SystemReset();
}
#endif

#if configUSE_MALLOC_FAILED_HOOK
void vApplicationMallocFailedHook(void){
#if CONFIG_USE_LOG_MONITOR
	LOG_ERROR(TAG, "Memory allocation fail.");
#endif /* CONFIG_USE_LOG_MONITOR */
	for(uint32_t i=0; i< 80000000; i++) __NOP();
	__NVIC_SystemReset();
}
#endif

#if (configSUPPORT_STATIC_ALLOCATION == 1)

__WEAK void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	static StaticTask_t Idle_TCB;
	static StackType_t  Idle_Stack[configMINIMAL_STACK_SIZE];

	*ppxIdleTaskTCBBuffer   = &Idle_TCB;
	*ppxIdleTaskStackBuffer = &Idle_Stack[0];
	*pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
}

__WEAK void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
	static StaticTask_t Timer_TCB;
	static StackType_t  Timer_Stack[configTIMER_TASK_STACK_DEPTH];

	*ppxTimerTaskTCBBuffer   = &Timer_TCB;
	*ppxTimerTaskStackBuffer = &Timer_Stack[0];
	*pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}
#endif





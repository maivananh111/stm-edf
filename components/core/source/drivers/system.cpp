/*
 * system.cpp
 *
 *  Created on: Aug 6, 2023
 *      Author: anh
 */
//#include CONFIG_CORE_HEADER_FILE
#include "drivers/system.h"
#include "drivers/embedflash.h"
#include "drivers/systick.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_LOG_MONITOR_ENABLE
#include "common/log_monitor.h"

static const char *Excep_TAG = "EXCEPTION";
static const char *Inter_TAG = "INTERRUPT";
static const char *Sys_TAG =   "SYSTEM";
#endif /* CONFIG_LOG_MONITOR_ENABLE */


static volatile void (*exception_hander)(void *param);
static void *exception_parameter = NULL;
static void (*delay_ms_func)(uint32_t) = systick_delay_ms;
static uint32_t (*get_tick_func)(void) = systick_get_tick;


void system_init(void){
#if CONFIG_FPU_ENABLE
	SystemInit();
#endif

	embedflash_init();

	__NVIC_SetPriorityGrouping(0x03U);

	systick_init();

	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
#if defined(STM32F4)
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	PWR -> CR |= PWR_CR_VOS;
#endif /* STM32F4 */
}

void system_reset(void){
#if CONFIG_SYS_CONFIG_FAIL_RESET_ENABLE
#if CONFIG_LOG_MONITOR_ENABLE
		LOG_INFO(Sys_TAG, "Chip will reset after %ds.", CONFIG_SYS_CONFIG_FAIL_RESET_TIME);
#endif /* CONFIG_PERIPH_SPI_LOG */
		delay_ms(CONFIG_SYS_CONFIG_FAIL_RESET_TIME*1000U);
		__NVIC_SystemReset();
#endif /* CONFIG_SYS_CONFIG_FAIL_RESET_ENABLE */
}


void system_add_exception_handler(volatile void(*exception_hander_function)(void *param), void *param){
	exception_hander = exception_hander_function;
	exception_parameter = param;
}

void exception_interrupt_handler(const char *tag, char *message){
#if CONFIG_LOG_MONITOR_ENABLE
	LOG_ERROR(tag, message);
	if(exception_hander != NULL) exception_hander(exception_parameter);
#endif /* CONFIG_USE_LOG_MONITOR */
}


uint32_t get_tick(void){
	return get_tick_func();
}

void delay_ms(uint32_t ms){
	delay_ms_func(ms);
}

void system_set_function_get_tick(uint32_t (*func_ptr)(void)){
	get_tick_func = func_ptr;
}

void system_set_function_delay_ms(void(*func_ptr)(uint32_t)){
	delay_ms_func = func_ptr;
}




void SysTick_Handler(void){
	systick_app_systick_process();

#if (INCLUDE_xTaskGetSchedulerState == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED){
#endif /* INCLUDE_xTaskGetSchedulerState */
		xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	}
#endif /* INCLUDE_xTaskGetSchedulerState */
}

void NMI_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Inter_TAG, (char *)"NonMaskable interrupt was unhandled(NMI_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
    while(1){}
}


void HardFault_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Excep_TAG, (char *)"Hard fault exception was unhandled(call HardFault_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
    while (1){}
}

void MemManage_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Inter_TAG, (char *)"Memory management interrupt was unhandled(MemManage_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
    while (1){}
}

void BusFault_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Excep_TAG, (char *)"Bus fault exception was unhandled(call BusFault_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
    while (1){}
}

void UsageFault_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Excep_TAG, (char *)"Usage fault exception was unhandled(call UsageFault_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
    while (1){}
}

void DebugMon_Handler(void){
#if CONFIG_LOG_MONITOR_ENABLE
	exception_interrupt_handler(Inter_TAG, (char *)"Debug monitor interrupt was unhandled(call DebugMon_Handler)...");
#endif /* CONFIG_LOG_MONITOR_ENABLE */
}





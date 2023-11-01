/*
 * system.h
 *
 *  Created on: Aug 6, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_SYSTEM_H_
#define PERIPHERALS_SYSTEM_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdio.h"

#ifdef __cplusplus
extern "C"{
#endif


void system_init(void);
void system_add_exception_handler(volatile void(*exception_hander_function)(void *param), void *param);
void exception_interrupt_handler(const char *tag, char *message);

uint32_t get_tick(void);
void delay_ms(uint32_t ms);

void system_set_function_get_tick(uint32_t (*func_ptr)(void));
void system_set_function_delay_ms(void(*func_ptr)(uint32_t));

void system_reset(void);

void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);


#ifdef __cplusplus
}
#endif


#endif /* PERIPHERALS_SYSTEM_H_S */

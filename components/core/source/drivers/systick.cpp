/*
 * systick.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */
#include "drivers/systick.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"


__IO uint32_t _systick_mtick;
extern __IO uint32_t _systick_total_ticks;
extern __IO uint32_t _systick_idle_ticks;
extern __IO float    _cpu_load_percent;


void systick_init(void){
	SysTick_Config(SystemCoreClock / CONFIG_PERIPH_SYSTICK_FREQUENCY);

	__NVIC_SetPriority(SysTick_IRQn, CONFIG_PERIPH_SYSTICK_INTERRUPT_PRIORITY);
}

inline void systick_increment_tick(void){
	_systick_mtick++;
}

uint32_t systick_get_tick(void){
	return _systick_mtick;
}

void systick_delay_ms(uint32_t ms){
	uint32_t tickstart = _systick_mtick;
	uint32_t wait = ms;

	if (wait < 0xFFFFFFU) wait += 1UL;

	while((_systick_mtick - tickstart) < wait);
}

extern "C"{
	void systick_app_systick_process(void){
		systick_increment_tick();

		_systick_total_ticks++;
		if(_systick_total_ticks == 1000){
			_cpu_load_percent = (float)(100.0 - (((float)_systick_idle_ticks / (float)_systick_total_ticks) * 100.0));
			_systick_total_ticks = 0;
			_systick_idle_ticks = 0;
		}
	}
}












/*
 * systick.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_SYSTICK_H_
#define PERIPHERALS_SYSTICK_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdio.h"

#ifdef __cplusplus
extern "C"{
#endif


void systick_init(void);

inline void systick_increment_tick(void);
void systick_app_systick_process(void);

uint32_t systick_get_tick(void);
void systick_delay_ms(uint32_t ms);


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_SYSTICK_H_ */

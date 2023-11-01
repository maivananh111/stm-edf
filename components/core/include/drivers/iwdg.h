/*
 * iwdg.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_IWDG_H_
#define PERIPHERALS_IWDG_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_IWDG_EN && defined(IWDG)
#define PERIPHERAL_IWDG_AVAILABLE 1

#include "common/error_check.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef struct{
	uint32_t prescaler = 0;
	uint32_t autoreload = 0;
} iwdg_config_t;


iwdg_config_t iwdg_cal_param(uint32_t wait_time);
err_t iwdg_init(iwdg_config_t *conf);

void iwdg_EN_in_debugmode(void);
void iwdg_disable_in_debugmode(void);

void iwdg_refresh(void);


#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_IWDG_AVAILABLE 0
#endif /* CONFIG_PERIPH_IWDG_EN && defined(IWDG) */

#endif /* PERIPHERALS_IWDG_H_ */

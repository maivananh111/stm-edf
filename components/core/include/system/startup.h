/*
 * main.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef STARTUP_H_
#define STARTUP_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "drivers/sysinfo.h"
#include "drivers/system.h"
#include "drivers/systick.h"
#include "drivers/rcc.h"
#include "drivers/gpio.h"

#include "common/macro.h"
#include "common/error_check.h"
#include "common/expression_check.h"
#include "common/bits_check.h"

#if CONFIG_MEM_SUPPORT_SDRAM
#include "drivers/fmc_sdram.h"
#endif /* CONFIG_MEM_SUPPORT_SDRAM */

#if CONFIG_WATCHDOG_EN
#include "drivers/iwdg.h"
#endif /* CONFIG_WATCHDOG_EN */

#if CONFIG_PERIPH_TRNG_EN
#include "drivers/trng.h"
#endif /* CONFIG_PERIPH_RNG_EN */

#if CONFIG_USE_LOG_MONITOR
#include "common/log_monitor.h"
#include "logconfig.h"
#endif /* CONFIG_USE_LOG_MONITOR */

#ifdef __cplusplus
extern "C"{
#endif


int edf_main_application(void);

#ifdef __cplusplus
}
#endif

#endif /* STARTUP_H_ */

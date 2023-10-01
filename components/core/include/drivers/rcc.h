/*
 * rcc.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_RCC_H_
#define PERIPHERALS_RCC_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE

#include "stdio.h"
#include "common/error_check.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	PLL_SOURCE_HSI,
	PLL_SOURCE_HSE,
} rcc_pllsource_t;

typedef enum{
	HSI_CRYSTAL,
	HSE_CRYSTAL,
	PLL_CLOCK,
} rcc_sysclocksource_t;

typedef enum{
	BUS_HCLK,
	BUS_APB1,
	BUS_APB2,
	BUS_SDIO_USB,
	BUS_APB1_TIMER,
	BUS_APB2_TIMER,
} rcc_busclock_t;

typedef struct{
	rcc_sysclocksource_t sysclk_source = HSI_CRYSTAL;
	rcc_pllsource_t pll_source = PLL_SOURCE_HSI;
	uint16_t hclk_freq_mhz = 16U;
	uint16_t apb1_freq_mhz = 16U;
	uint16_t apb2_freq_mhz = 16U;
	uint16_t sdio_usb_freq_mhz = 48U;
} rcc_config_t;

typedef struct{
#if defined(STM32F1)
	uint16_t hse_div = 2U;
	uint16_t pllmul = 9U;
	uint16_t usb_div = 0U;
#elif defined(STM32F4)
	uint16_t pllm = 16U;
	uint16_t plln = 192U;
	uint16_t pllp = 2U;
	uint16_t pllq = 4U;
#endif
	uint16_t hclk_div = 1U;
	uint16_t apb1_div = 1U;
	uint16_t apb2_div = 1U;

} rcc_param_t;



err_t rcc_calculate(rcc_config_t *conf, rcc_param_t *param);

err_t rcc_init(rcc_config_t *conf);
err_t rcc_deinit(void);

uint32_t rcc_get_bus_frequency(rcc_busclock_t bus);



#ifdef __cplusplus
}
#endif


#endif /* PERIPHERALS_RCC_H_ */

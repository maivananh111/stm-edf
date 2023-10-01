/*
 * rcc.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "drivers/rcc.h"

#include "drivers/embedflash.h"
#include "drivers/systick.h"
#include "common/bits_check.h"
#include "common/macro.h"
#include "math.h"



#define RCC_HSI_TIMEOUT          100U
#define RCC_HSE_TIMEOUT          200U
#define RCC_PLL_TIMEOUT          100U
#define RCC_SWS_TIMEOUT 	 	 5000U


#define HSE_FREQ CONFIG_CLK_HSE_FREQ
#define HSI_FREQ CONFIG_CLK_HSI_FREQ
#define LSE_FREQ CONFIG_CLK_LSE_FREQ
#define LSI_FREQ CONFIG_CLK_LSI_FREQ

static rcc_config_t *_conf;
static rcc_param_t _param;
static const uint8_t AHBDiv[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8_t APBDiv[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


static uint8_t hclk_div_cal(uint16_t div);
static uint8_t apbx_div_cal(uint16_t div);
static void rcc_update_system_clock(void);

static void rcc_update_system_clock(void){
#if defined(STM32F4)
	uint32_t pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
#endif
	uint32_t tmp = 0;
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp){
		case 0x00:  /* HSI used as system clock source */
			SystemCoreClock = HSI_FREQ * 1000000U;
		break;
		case 0x04:  /* HSE used as system clock source */
			SystemCoreClock = HSE_FREQ * 1000000U;
		break;
		case 0x08:  /* PLL used as system clock source */
#if defined(STM32F1)

#elif defined(STM32F4)
			pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
			pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

			if (pllsource != 0)
				pllvco = (HSE_FREQ * 1000000U / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			else
				pllvco = (HSI_FREQ * 1000000U / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);

			pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1 ) *2;
			SystemCoreClock = pllvco/pllp;
#endif
		break;
		default:
			SystemCoreClock = HSI_FREQ * 1000000U;
		break;
	}
	tmp = AHBDiv[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	SystemCoreClock >>= tmp;
}


static uint8_t hclk_div_cal(uint16_t div){
	switch(div){
		case 2:
			return 8;
		break;
		case 4:
			return 9;
		break;
		case 8:
			return 10;
		break;
		case 16:
			return 11;
		break;
		case 64:
			return 12;
		break;
		case 128:
			return 13;
		break;
		case 256:
			return 14;
		break;
		case 512:
			return 15;
		break;
		default:
			return 7;
		break;
	}
	return 0;
}

static uint8_t apbx_div_cal(uint16_t div){
	switch(div){
		case 2:
			return 4;
		break;
		case 4:
			return 5;
		break;
		case 8:
			return 6;
		break;
		case 16:
			return 7;
		break;
		default:
			return 3;
		break;
	}
	return 0;
}


err_t rcc_calculate(rcc_config_t *conf, rcc_param_t *param){
    __IO float Fxtal_MHz, Fhclk_MHz, Fapb1_MHz, Fapb2_MHz, Fusb_MHz;
    __IO uint16_t hclk_div, apb1_div, apb2_div;
#if defined(STM32F1)
    __IO float xtal_MHz;
	const uint16_t hsi_div = 2U;
    uint16_t mul, cry_div, usb_div;
    float usb_div_arr[2] = {1.5, 1.0};
#elif defined(STM32F4)
    __IO uint16_t  m, n, p, q;
#endif

    if(conf->pll_source == PLL_SOURCE_HSE) {
    	Fxtal_MHz = (float)HSE_FREQ;
    	if(Fxtal_MHz < CONFIG_CLK_MIN_HSE_FREQ || Fxtal_MHz > CONFIG_CLK_MAX_HSE_FREQ) return {E_INVALID, CODE_LINE};
    }
    else {
    	Fxtal_MHz = (float)HSI_FREQ;
    }

    Fhclk_MHz = (float)conf->hclk_freq_mhz;
    Fapb1_MHz = (float)conf->apb1_freq_mhz;
    Fapb2_MHz = (float)conf->apb2_freq_mhz;
    Fusb_MHz  = (float)conf->sdio_usb_freq_mhz;
    if(Fhclk_MHz > CONFIG_CLK_MAX_HCLK_FREQ) return {E_INVALID, CODE_LINE};
    if(Fapb1_MHz > CONFIG_CLK_MAX_APB1_FREQ) return {E_INVALID, CODE_LINE};
    if(Fapb2_MHz > CONFIG_CLK_MAX_APB2_FREQ) return {E_INVALID, CODE_LINE};
    if(Fusb_MHz > CONFIG_CLK_MAX_SDIO_USB_FREQ) return {E_INVALID, CODE_LINE};

#if defined(STM32F1)
    for(cry_div = 1; cry_div<=2; cry_div++){
    	if(conf->pll_source == PLL_SOURCE_HSI) cry_div = hsi_div;
    	xtal_MHz = Fxtal_MHz / cry_div;
		for(mul = 2; mul <= 16; mul++){
			for(hclk_div = 1; hclk_div <= 256; hclk_div *= 2) {
				if(hclk_div != 32){
					__IO float hclk_MHz = (xtal_MHz * mul) / hclk_div;
					if(fabs(hclk_MHz - Fhclk_MHz) <= 0.00001){
						goto cal_usb_div;
					}
				}
			}
		}
    }

#elif defined(STM32F4)
    for (m = Fxtal_MHz; m >= 2; m--) {
        __IO float pllm_MHz = Fxtal_MHz / m;
        if (pllm_MHz >= CONFIG_CLK_MIN_M_OFREQ && pllm_MHz <= CONFIG_CLK_MAX_M_OFREQ) {
            for (n = 432; n >= 50; n--) {
                __IO float plln_MHz = pllm_MHz * n;
                if (plln_MHz >= CONFIG_CLK_MIN_N_OFREQ && plln_MHz <= CONFIG_CLK_MAX_N_OFREQ) {
                    for (p = 2; p <= 8; p += 2) {
                        __IO float pllp_MHz = plln_MHz / p;
                        if (pllp_MHz >= CONFIG_CLK_MIN_P_OFREQ && pllp_MHz <= CONFIG_CLK_MAX_P_OFREQ) {
							for (hclk_div = 1; hclk_div <= 256; hclk_div *= 2) {
								if(hclk_div != 32){
									__IO float cpu_hclk_MHz = pllp_MHz / hclk_div;
									if(fabs(cpu_hclk_MHz - Fhclk_MHz) <= 0.0001) {
										goto cal_pllq;
									}
								}
							}
						}
                    }
                }
            }
        }
    }
    goto error_resolve;

    cal_pllq:
	for (q = 2; q <= 15; q++) {
		float pllq_MHz = Fxtal_MHz / m * n / q;
		if (pllq_MHz <= Fusb_MHz) {
			goto cal_apb1_div;
		}
	}
#endif
#if defined(STM32F1)
	goto error_resolve;
	cal_usb_div:
	xtal_MHz = Fxtal_MHz / cry_div;
	for(usb_div = 0; usb_div<=1; usb_div++){
		__IO float usb_MHz = (xtal_MHz * mul) / usb_div_arr[usb_div];
		if(usb_MHz <= Fusb_MHz){
			goto cal_apb1_div;
		}
	}
#endif
	goto error_resolve;
	cal_apb1_div:
	for(apb1_div = 1; apb1_div <= 16; apb1_div *= 2) {
		float apb1_MHz = Fhclk_MHz / apb1_div;
		if (apb1_MHz <= Fapb1_MHz) {
			goto cal_apb2_div;
		}
	}
	goto error_resolve;
	cal_apb2_div:
	for (apb2_div = 1; apb2_div <= 16; apb2_div *= 2) {
		float apb2_MHz = Fhclk_MHz / apb2_div;
		if (apb2_MHz <= Fapb2_MHz) {
			goto resolved;
		}
	}
	goto error_resolve;


	resolved:
#if defined(STM32F1)
	param->hse_div = cry_div - 1U;
	param->pllmul = mul;
	param->usb_div = usb_div;
#elif defined(STM32F4)
	param->pllm = m;
	param->plln = n;
	param->pllp = p;
	param->pllq = q;
#endif
	param->hclk_div = hclk_div;
	param->apb1_div = apb1_div;
	param->apb2_div = apb2_div;

	return {E_PASS, 0};

error_resolve:
    return {E_FAIL, 0};
}

err_t rcc_init(rcc_config_t *conf){
	err_t ret;
	__IO uint32_t tmpreg = 0;
	_conf = conf;
	/**
	 * OSC Configuration.
	 */

#if defined(STM32F1)
	if((RCC->CFGR & RCC_CFGR_SWS_HSE) || ((RCC->CFGR & RCC_CFGR_SWS_PLL) && (RCC->CFGR & RCC_CFGR_PLLSRC))){
#elif defined(STM32F4)
	if((RCC->CFGR & RCC_CFGR_SWS_HSE) || ((RCC->CFGR & RCC_CFGR_SWS_PLL) && (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE))){
#endif /* STM32F4 */
		if(!(RCC->CR & RCC_CR_HSERDY)){
			ERROR_SET(ret, E_FAIL);
			return ret;
		}
	}

	if(_conf->sysclk_source == HSI_CRYSTAL || _conf->pll_source == PLL_SOURCE_HSI){
		RCC->CR |= RCC_CR_HSION;
		ret = wait_bits(&(RCC->CR), RCC_CR_HSIRDY, BITS_SET, RCC_HSI_TIMEOUT);
		IS_E_TIMEOUT(ret){
			ERROR_CAPTURE(ret);
			return ret;
		}
		RCC->CR &= ~RCC_CR_HSITRIM_Msk;
		RCC->CR |= (CONFIG_CLK_HSI_TRIM_VALUE << RCC_CR_HSITRIM_Pos);

	}
	else if(_conf->sysclk_source == HSE_CRYSTAL || _conf->pll_source == PLL_SOURCE_HSE){
		RCC->CR |= RCC_CR_HSEON;
		ret = wait_bits(&(RCC->CR), RCC_CR_HSERDY, BITS_SET, RCC_HSE_TIMEOUT);
		IS_E_TIMEOUT(ret){
			ERROR_CAPTURE(ret);
			return ret;
		}
	}
	else{
		ERROR_SET(ret, E_FAIL);
		return ret;
	}

	/**
	 * PLL Configuration.
	 */
	RCC->CR &=~ RCC_CR_PLLON;
	ret = wait_bits(&(RCC->CR), RCC_CR_PLLRDY, BITS_RESET, RCC_PLL_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	ret = rcc_calculate(_conf, &_param);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
#if defined(STM32F1)
	if(_param.pllmul > 16UL) _param.pllmul = 16UL;
	_param.pllmul = _param.pllmul - 2U;
	tmpreg = RCC->CFGR;
	tmpreg &=~ (RCC_CFGR_PLLMULL_Msk | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_USBPRE);
	tmpreg |= (_param.pllmul << RCC_CFGR_PLLMULL_Pos);
	tmpreg |= (_conf->pll_source << RCC_CFGR_PLLSRC_Pos);
	tmpreg |= (_param.hse_div << RCC_CFGR_PLLXTPRE_Pos);
	tmpreg |= (_param.usb_div << RCC_CFGR_USBPRE_Pos);
	RCC->CFGR = tmpreg;
#elif defined(STM32F4)
	tmpreg = RCC->PLLCFGR;
	tmpreg &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLSRC_Msk);
	tmpreg |= ((_param.pllm) << RCC_PLLCFGR_PLLM_Pos) | ((_param.plln) << RCC_PLLCFGR_PLLN_Pos) | ((((_param.pllp) >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
			  ((_param.pllq) << RCC_PLLCFGR_PLLQ_Pos) | ((_conf->pll_source) << RCC_PLLCFGR_PLLSRC_Pos);
	RCC->PLLCFGR = tmpreg;
#endif /* STM32F4 */
	RCC->CR |= RCC_CR_PLLON;
	ret = wait_bits(&(RCC->CR), RCC_CR_PLLRDY, BITS_SET, RCC_PLL_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	/**
	 * Calculation flash latency and update latency if new latency great than current latency.
	 */
	uint32_t latency = embedflash_calculate_latency(_conf->hclk_freq_mhz * 1000000UL);
	uint32_t current_latency = embedflash_get_latency();
	if(latency > current_latency) embedflash_set_latency(latency);

	/**
	 * Check over clock to enable Over-drive mode.
	 */
#if CONFIG_CLK_OVER_CLOCK && defined(STM32F4)
	PWR->CR |= PWR_CR_ODEN;
	ret = wait_bits(&(PWR->CSR), PWR_CSR_ODRDY, BITS_SET, 1000U);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	PWR->CR |= PWR_CR_ODSWEN;
	ret = wait_bits(&(PWR->CSR), PWR_CSR_ODSWRDY, BITS_SET, 1000U);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
#endif /* CONFIG_CLK_SUPPORT_OVER_CLOCK */

	/**
	 * Check and set system clock source.
	 */
	if(_conf->sysclk_source == HSI_CRYSTAL){
		if(!(RCC->CR & RCC_CR_HSIRDY)){
			ERROR_CAPTURE(ret);
			return ret;
		}
	}
	else if(_conf->sysclk_source == HSE_CRYSTAL){
		if(!(RCC->CR & RCC_CR_HSERDY)){
			ERROR_CAPTURE(ret);
			return ret;
		}
	}
	else if(_conf->sysclk_source == PLL_CLOCK){
		if(!(RCC->CR & RCC_CR_PLLRDY)){
			ERROR_CAPTURE(ret);
			return ret;
		}
	}

	RCC->CFGR = ((RCC->CFGR & !RCC_CFGR_SW_Msk) | (_conf->sysclk_source << RCC_CFGR_SW_Pos));
	ret = wait_bits(&(RCC->CFGR), (_conf->sysclk_source << RCC_CFGR_SW_Pos), BITS_SET, RCC_SWS_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	/**
	 * SYSCLK, AHB, APB1, APB2 frequency configuration.
	 */
	tmpreg = RCC->CFGR;
	tmpreg &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
	tmpreg |= (hclk_div_cal(_param.hclk_div) << RCC_CFGR_HPRE_Pos) | (apbx_div_cal(_param.apb1_div) << RCC_CFGR_PPRE1_Pos) | (apbx_div_cal(_param.apb2_div) << RCC_CFGR_PPRE2_Pos);
	RCC->CFGR = tmpreg;

	/**
	 * Update system core clock..
	 */
	rcc_update_system_clock();

	/**
	 * Update latency if new latency less than current latency.
	 */
	if(latency < current_latency) embedflash_set_latency(latency);

	/**
	 * ReInit system tick.
	 */
	systick_init();

#if defined(STM32F1)
	/** ENABLE AFIO CLOCK */
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	/** DISABLE JTAG */
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

#endif /* STM32F1 */

	return ret;
}

err_t rcc_deinit(void){
	err_t ret;

	/**
	 * Turn on HSI (default).
	 */
	RCC->CR |= RCC_CR_HSION;
	ret = wait_bits(&(RCC->CR), RCC_CR_HSIRDY, BITS_SET, RCC_HSI_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
	RCC->CR &= ~RCC_CR_HSITRIM_Msk;
	RCC->CR |= (0x10U << RCC_CR_HSITRIM_Pos);

	/**
	 * Clear CFGR register.
	 */
	RCC->CFGR = 0x00U;
	ret = wait_bits(&(RCC->CFGR), 0xFFFFFFFFU, BITS_RESET, RCC_SWS_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	/**
	 * Turn off HSE.
	 */
	RCC->CR &=~ RCC_CR_HSEON;
	ret = wait_bits(&(RCC->CR), RCC_CR_HSERDY, BITS_RESET, RCC_HSE_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	/**
	 * Turn off PLL.
	 */
	RCC->CR &=~ RCC_CR_PLLON;
	ret = wait_bits(&(RCC->CR), RCC_CR_PLLRDY, BITS_RESET, RCC_PLL_TIMEOUT);
	IS_E_TIMEOUT(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
#if defined(STM32F1)
	RCC->CFGR &=~ (RCC_CFGR_PLLMULL_Msk);
#elif defined(STM32F4)
	RCC->PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;
#endif /* STM32F4 */

	rcc_update_system_clock();

	/**
	 * ReInit system tick.
	 */
	systick_init();


	return ret;
}

uint32_t rcc_get_bus_frequency(rcc_busclock_t bus){
	switch(bus){
		case BUS_HCLK:
			if(_conf->sysclk_source == HSE_CRYSTAL) // HSE.
				return (uint32_t)HSE_FREQ * 1000000U / _param.hclk_div;
			if(_conf->sysclk_source == HSI_CRYSTAL) // HSI.
				return (uint32_t)HSI_FREQ * 1000000U / _param.hclk_div;
			else{
				if(_conf->pll_source == PLL_SOURCE_HSE){
#if defined(STM32F1)
					return (uint32_t)(HSE_FREQ * 1000000U / (_param.hse_div+1U) * (_param.pllmul + 2U) / _param.hclk_div);
#elif defined(STM32F4)
					return (uint32_t)(HSE_FREQ * 1000000U / _param.pllm * _param.plln / _param.pllp / _param.hclk_div);
#endif /* STM32F4 */
				}
				else{ // HSI.
#if defined(STM32F1)
					return (uint32_t)((HSI_FREQ * 1000000U / 2U) * (_param.pllmul + 2U) / _param.hclk_div);
#elif defined(STM32F4)
					return (uint32_t)(HSI_FREQ * 1000000U / _param.pllm * _param.plln / _param.pllp / _param.hclk_div);
#endif /* STM32F4 */
				}
			}
		break;

		case BUS_APB1:
			return (uint32_t)(SystemCoreClock >> APBDiv[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
		break;

		case BUS_APB2:
			return (uint32_t)(SystemCoreClock >> APBDiv[(RCC->CFGR & RCC_CFGR_PPRE2)>> RCC_CFGR_PPRE2_Pos]);
		break;

		case BUS_SDIO_USB:
			if(_conf->pll_source == PLL_SOURCE_HSE){
#if defined(STM32F1)
				return (uint32_t)(HSE_FREQ * 1000000U / (_param.hse_div+1U) * _param.pllmul / (1.5 - _param.usb_div));
#elif defined(STM32F4)
				return (uint32_t)(HSE_FREQ * 1000000U / _param.pllm * _param.plln / _param.pllq);
#endif
			}
			else{
#if defined(STM32F1)
				return (uint32_t)(HSI_FREQ * 1000000U / 2U * _param.pllmul / (1.5 - _param.usb_div));
#elif defined(STM32F4)
				return (uint32_t)(HSI_FREQ * 1000000U / _param.pllm * _param.plln / _param.pllq);
#endif
			}
		break;

		case BUS_APB1_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBDiv[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]));
		break;

		case BUS_APB2_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBDiv[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]));
		break;

	}
	return 0;
}










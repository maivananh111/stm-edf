/*
 * embedflash.cpp
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */


#include "drivers/embedflash.h"
#if PERIPHERAL_EMBEDFLASH_AVAILABLE



void embedflash_init(void){
#if defined(STM32F1)
	/* FLASH LATENCY 2WS, PREFETCH BUFER ENABLE, DATA CACHE ENABLE */
	FLASH->ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;
	while(!(FLASH->ACR & FLASH_ACR_PRFTBS));
#elif defined(STM32F4)
#if CONFIG_MEM_FLASH_INSTRUCTION_CACHE_ENABLE
	SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
#else
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
#endif

#if CONFIG_MEM_FLASH_DATA_CACHE_ENABLE
	SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);

#else
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN);
#endif

#if CONFIG_MEM_FLASH_PREFETCH_ENABLE
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
#else
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
#endif
#endif /* STM32F4 */
}

uint32_t embedflash_calculate_latency(uint32_t freq_hz){
#if defined(STM32F1)
	uint32_t latency= (uint32_t)(freq_hz / 24000000U);
	if(freq_hz == 24000000U || freq_hz == 48000000U || freq_hz == 72000000U) latency -= 1;
#elif defined(STM32F4)
	uint32_t latency= (uint32_t)(freq_hz / 30000000U);
	if(freq_hz == 30000000U || freq_hz == 60000000U || freq_hz == 90000000U
    || freq_hz == 120000000U || freq_hz == 150000000U || freq_hz == 180000000U) latency -= 1;
#endif /* STM32F4 */

	return latency;
}

void embedflash_set_latency(uint32_t latency){
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, (latency << FLASH_ACR_LATENCY_Pos));
}

void embedflash_update_latency(void){
	uint32_t tmpreg = (FLASH->ACR & (~FLASH_ACR_LATENCY_Msk));

	uint32_t latency= (uint32_t)(SystemCoreClock / 30000000U);
	if(SystemCoreClock == 30000000U || SystemCoreClock == 60000000U || SystemCoreClock == 90000000U
    || SystemCoreClock == 120000000U || SystemCoreClock == 150000000U || SystemCoreClock == 180000000U) latency -= 1;

	tmpreg |= (uint32_t)(latency << FLASH_ACR_LATENCY_Pos);
	SET_BIT(FLASH->ACR, tmpreg);
}

uint32_t embedflash_get_latency(void){
	return (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
}




#endif /* PERIPHERAL_EMBEDFLASH_AVAILABLE */


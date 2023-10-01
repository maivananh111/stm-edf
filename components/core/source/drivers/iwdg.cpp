/*
 * iwdg.cpp
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */
#include "drivers/iwdg.h"
#if PERIPHERAL_IWDG_AVAILABLE

#include "common/bits_check.h"

#define IWDG_KEY_RELOAD                 0x0000AAAAu
#define IWDG_KEY_ENABLE                 0x0000CCCCu
#define IWDG_KEY_WRITE_ACCESS_ENABLE    0x00005555u
#define IWDG_KEY_WRITE_ACCESS_DISABLE   0x00000000u
#define IWDG_DEFAULT_TIMEOUT            (((6UL*256UL*1000UL) / LSI_VALUE) + ((LSI_STARTUP_TIME/1000UL)+1UL))

#define IWDG_PRESCALER_4                0x00000000U
#define IWDG_PRESCALER_8                IWDG_PR_PR_0
#define IWDG_PRESCALER_16               IWDG_PR_PR_1
#define IWDG_PRESCALER_32               (IWDG_PR_PR_1 | IWDG_PR_PR_0)
#define IWDG_PRESCALER_64               IWDG_PR_PR_2
#define IWDG_PRESCALER_128              (IWDG_PR_PR_2 | IWDG_PR_PR_0)
#define IWDG_PRESCALER_256              (IWDG_PR_PR_2 | IWDG_PR_PR_1)


static iwdg_config_t *_conf;

iwdg_config_t iwdg_cal_param(uint32_t wait_time){
	iwdg_config_t _conf;

	_conf.autoreload = wait_time;
	_conf.prescaler = IWDG_PRESCALER_32;

	return _conf;
}

err_t iwdg_init(iwdg_config_t *conf){
	err_t ret;
	_conf = conf;

	IWDG -> KR = IWDG_KEY_ENABLE;

	IWDG -> KR = IWDG_KEY_WRITE_ACCESS_ENABLE;

	IWDG -> PR = _conf->prescaler;
	IWDG -> RLR = _conf->autoreload - 1;

	ret = wait_bits(&(IWDG->SR), (IWDG_SR_RVU | IWDG_SR_PVU), BITS_RESET, IWDG_DEFAULT_TIMEOUT);

	IWDG -> KR = IWDG_KEY_RELOAD;

	return ret;
}

void iwdg_enable_in_debugmode(void){
#if defined(STM32F1)
	DBGMCU->CR &=~ DBGMCU_CR_DBG_IWDG_STOP;
#elif defined(STM32F4)
	DBGMCU->APB1FZ &=~ DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif /* STM32F4 */
}

void iwdg_disable_in_debugmode(void){
#if defined(STM32F1)
	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
#elif defined(STM32F4)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif /* STM32F4 */
}

void iwdg_refresh(void){
	IWDG -> KR = IWDG_KEY_RELOAD;
}



#endif /* PERIPHERAL_IWDG_AVAILABLE */

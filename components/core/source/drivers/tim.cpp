/*
 * tim.cpp
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */


#include "drivers/tim.h"
#if PERIPHERAL_TIM_AVAILABLE
#include "drivers/rcc.h"
#include "drivers/system.h"
#include "drivers/systick.h"
#if CONFIG_PERIPH_TIM_LOG
#include "common/log_monitor.h"
#endif /* CONFIG_PERIPH_TIM_LOG */



static const char *TAG = "TIM";

#ifdef CONFIG_PERIPH_TIM_LOG
#define TIM_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, TAG, __MSG__);
#else
#define TIM_DBG(__MSG__)\
{}
#endif

static inline void TIMCommon_IRQHandler(tim *ptim);

tim::tim(TIM_TypeDef *tim){
	_instance = tim;
}

void tim::hardware_initialize(void){
#if defined(TIM1)
	if     (_instance == TIM1){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
		_IRQn = TIM1_UP_TIM10_IRQn;
	}
#endif
#if defined(TIM2)
	else if(_instance == TIM2){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
		_IRQn = TIM2_IRQn;
	}
#endif
#if defined(TIM3)
	else if(_instance == TIM3){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
		_IRQn = TIM3_IRQn;
	}
#endif
#if defined(TIM4)
	else if(_instance == TIM4){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
		_IRQn = TIM4_IRQn;
	}
#endif
#if defined(TIM5)
	else if(_instance == TIM5){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
		_IRQn = TIM5_IRQn;
	}
#endif
#if defined(TIM6)
	else if(_instance == TIM6){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
		_IRQn = TIM6_DAC_IRQn;
	}
#endif
#if defined(TIM7)
	else if(_instance == TIM7){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
		_IRQn = TIM7_IRQn;
	}
#endif
#if defined(TIM8)
	else if(_instance == TIM8){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
		_IRQn = TIM8_UP_TIM13_IRQn;
	}
#endif
#if defined(TIM9)
	else if(_instance == TIM9){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
		_IRQn = TIM1_BRK_TIM9_IRQn;
	}
#endif
#if defined(TIM10)
	else if(_instance == TIM10){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN);
		_IRQn = TIM1_UP_TIM10_IRQn;
	}
#endif
#if defined(TIM11)
	else if(_instance == TIM11){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
		_IRQn = TIM1_TRG_COM_TIM11_IRQn;
	}
#endif
#if defined(TIM12)
	else if(_instance == TIM12){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN);
		_IRQn = TIM8_BRK_TIM12_IRQn;
	}
#endif
#if defined(TIM13)
	else if(_instance == TIM13){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN);
		_IRQn = TIM8_UP_TIM13_IRQn;
	}
#endif
#if defined(TIM14)
	else if(_instance == TIM14){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
		_IRQn = TIM8_TRG_COM_TIM14_IRQn;
	}
#endif
}

void tim::hardware_deinitialize(void){
#if defined(TIM1)
	if     (_instance == TIM1) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
#endif
#if defined(TIM2)
	else if(_instance == TIM2) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
#endif
#if defined(TIM3)
	else if(_instance == TIM3) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
#endif
#if defined(TIM4)
	else if(_instance == TIM4) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
#endif
#if defined(TIM5)
	else if(_instance == TIM5) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
#endif
#if defined(TIM6)
	else if(_instance == TIM6) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
#endif
#if defined(TIM7)
	else if(_instance == TIM7) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
#endif
#if defined(TIM8)
	else if(_instance == TIM8) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
#endif
#if defined(TIM9)
	else if(_instance == TIM9) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
#endif
#if defined(TIM10)
	else if(_instance == TIM10) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN);
#endif
#if defined(TIM11)
	else if(_instance == TIM11) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
#endif
#if defined(TIM12)
	else if(_instance == TIM12) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN);
#endif
#if defined(TIM13)
	else if(_instance == TIM13) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN);
#endif
#if defined(TIM14)
	else if(_instance == TIM14) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
#endif
}

/* TIM Base */
err_t tim::base_initialize(tim_config_t *conf){
	err_t ret;

	_conf = conf;

	hardware_initialize();

	_instance->CR1 = 0U;
	_instance->CR1 |= (_conf->direction << TIM_CR1_DIR_Pos) | (_conf->autoreloadpreload << TIM_CR1_ARPE_Pos) | (_conf->align << TIM_CR1_CMS_Pos);

	_instance->ARR = 0U;
	_instance->ARR = _conf->reload - 1;
	_instance->PSC = 0U;
	_instance->PSC = _conf->prescaler - 1;

	_instance->EGR = TIM_EGR_UG;

	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
		TIM_DBG("Interrupt priority out of range");
		system_reset();
		return ret;
	}
	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);

#if defined(STM32F4)
	if(_instance == TIM1 || _instance == TIM2) 										            _gpio_func = AF1_TIM1_2;
	else if(_instance == TIM3 || _instance == TIM4 || _instance == TIM5) 					    _gpio_func = AF2_TIM3_5;
	else if(_instance == TIM8 || _instance == TIM9 || _instance == TIM10 || _instance == TIM11) _gpio_func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				    _gpio_func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				    _gpio_func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		ERROR_SET(ret, E_NOT_SUPPORTED);
		return ret;
	}
#endif

	return ret;
}

void tim::base_deinitialize(void){
	hardware_deinitialize();

	CLEAR_REG(_instance->CR1);
	CLEAR_REG(_instance->ARR);
	CLEAR_REG(_instance->PSC);
	CLEAR_REG(_instance->EGR);
}

tim_config_t *tim::get_config(void){
	return _conf;
}

void tim::register_event_handler(tim_event_handler_f event_handler_function, void *param){
	_event_handler = event_handler_function;
	_event_parameter = param;
}

void tim::unregister_event_handler(void){
	_event_handler = NULL;
	_event_parameter = NULL;
}


#if PERIPHERAL_DMAC_AVAILABLE
void tim::link_dmac(dmac_t dmac_udp, dmac_t dmac_ch1, dmac_t dmac_ch2, dmac_t dmac_ch3, dmac_t dmac_ch4){
	_dmac_upd = dmac_udp;
	_dmac_ch1 = dmac_ch1;
	_dmac_ch2 = dmac_ch2;
	_dmac_ch3 = dmac_ch3;
	_dmac_ch4 = dmac_ch4;

	if(_dmac_upd_conf != NULL) free(_dmac_upd_conf);
	if(_dmac_ch1_conf != NULL) free(_dmac_ch1_conf);
	if(_dmac_ch2_conf != NULL) free(_dmac_ch2_conf);
	if(_dmac_ch3_conf != NULL) free(_dmac_ch3_conf);
	if(_dmac_ch4_conf != NULL) free(_dmac_ch4_conf);

	if(_dmac_upd != NULL){
		_dmac_upd_conf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
		_dmac_upd_conf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;
		_dmac_upd->initialize(_dmac_upd_conf);
		_dmac_upd->register_event_handler(FUNC_BIND(&tim::dmaupd_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
	if(_dmac_ch1 != NULL){
		_dmac_ch1_conf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
		_dmac_ch1_conf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;
		_dmac_ch1->initialize(_dmac_ch1_conf);
		_dmac_ch1->register_event_handler(FUNC_BIND(&tim::dmach1_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
	if(_dmac_ch2 != NULL){
		_dmac_ch2_conf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
		_dmac_ch2_conf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;
		_dmac_ch2->initialize(_dmac_ch2_conf);
		_dmac_ch2->register_event_handler(FUNC_BIND(&tim::dmach2_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
	if(_dmac_ch3 != NULL){
		_dmac_ch3_conf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
		_dmac_ch3_conf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;
		_dmac_ch3->initialize(_dmac_ch3_conf);
		_dmac_ch3->register_event_handler(FUNC_BIND(&tim::dmach3_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
	if(_dmac_ch4 != NULL){
		_dmac_ch4_conf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
		_dmac_ch4_conf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;
		_dmac_ch4->initialize(_dmac_ch4_conf);
		_dmac_ch4->register_event_handler(FUNC_BIND(&tim::dmach4_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
}
#endif



inline void tim::set_prescaler(uint32_t prescaler){
	_conf->prescaler = prescaler;
	_instance->PSC = prescaler;
}
inline void tim::set_autoreload(uint32_t autoreload){
	_conf->reload = autoreload;
	_instance->ARR = autoreload;
}

inline void tim::reset_counter(void){
	_instance->CNT = 0;
}

inline uint32_t tim::get_counter(void){
	return _instance->CNT;
}

void tim::delay_us(uint32_t us){
	_instance->CNT = 0;
	while(_instance->CNT < us);
}

void tim::delay_ms(uint32_t ms){
	while(ms--) delay_us(1000);
}

inline void tim::clear_update_isr(void){
	_instance->SR &=~ TIM_DIER_UIE;
}

inline void tim::enable_interrupt(void){
	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_EnableIRQ(_IRQn);
	__NVIC_ClearPendingIRQ(_IRQn);
}

inline void tim::disable_interrupt(void){
	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);
}

/**
 * Timer base function.
 */
inline void tim::base_start(void){
	_instance->CR1 |= TIM_CR1_CEN;
}

inline void tim::base_stop(void){
	_instance->CR1 &=~ TIM_CR1_CEN;
}

inline void tim::base_start_it(void){
	_instance->CR1 &=~ TIM_CR1_CEN;

	_instance->DIER |= TIM_DIER_UIE;
	enable_interrupt();

	_instance->CR1 |= TIM_CR1_CEN;

}

inline void tim::base_stop_it(void){
	_instance->CR1 &=~ TIM_CR1_CEN;
	_instance->DIER &=~ TIM_DIER_UIE;
	disable_interrupt();
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::base_start_dmac(uint32_t *parrbuff, uint16_t size){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_dmac_upd == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "TIM DMA invalid");
	ASSERT_THEN_RETURN_VALUE((parrbuff == NULL || size == 0U),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "Invalid parameter");

	_instance->CR1  &=~ TIM_CR1_CEN;

	struct dmac_session_config_t xfer_conf = {
		.psource = (uint32_t)parrbuff,
		.pdest = (uint32_t)&_instance->ARR,
		.xfersize = size,
	};
	ret = _dmac_upd->config_start_session(xfer_conf);
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		TIM_DBG("TIM DMA start error");
		_dmac_upd->stop_session();
		return ret;
	}
	_instance->DIER |= TIM_DIER_UDE;
	_instance->CR1  |= TIM_CR1_CEN;

	return ret;
}

err_t tim::base_stop_dmac(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_dmac_upd == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "TIM DMA invalid");

	_dmac_upd->stop_session();;
	_instance->CR1  &=~ TIM_CR1_CEN;

	return ret;
}
#endif


/**
 * Timer PWM output function.
 */
err_t tim::pwm_output_initialize(tim_channel_t channel, tim_config_pwm_t *conf){
	err_t ret;
	gpio_pin_t pwmpin;

	if(gpio_str_decode(conf->pin, &pwmpin) == false) return {E_INVALID, CODE_LINE};

	gpio_port_clock_enable(pwmpin.port);
#if defined(STM32F1)
	gpio_set_direction(pwmpin.port, pwmpin.pinnum, GPIO_FUNCTION);
	gpio_set_type(pwmpin.port, pwmpin.pinnum, GPIO_FUNCTION_PUSHPULL);
#elif defined(STM32F4)
	gpio_set_direction(pwmpin.port, pwmpin.pinnum, GPIO_FUNCTION);
	gpio_set_function(pwmpin.port, pwmpin.pinnum, _gpio_func);
	gpio_set_type(pwmpin.port, pwmpin.pinnum, GPIO_FUNCTION_PUSHPULL);
#endif /* STM32F4 */

	if(channel < TIM_CHANNEL3){ // Channel 1-2
		_instance->CCMR1 &=~ (0xFF << (channel*8));
		_instance->CCMR1 &=~ (TIM_CCMR1_CC1S << (channel*8));
		_instance->CCMR1 |= ((conf->preload << TIM_CCMR1_OC1PE_Pos) << (channel*8));
		_instance->CCMR1 |= ((conf->fastmode << TIM_CCMR1_OC1FE_Pos) << (channel*8));
		_instance->CCMR1 |= ((conf->invert << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){ // Channel 3-4
		_instance->CCMR2 &=~ (0xFF << ((channel-2)*8));
		_instance->CCMR2 &=~ (TIM_CCMR2_CC3S << ((channel-2)*8));
		_instance->CCMR2 |= ((conf->preload << TIM_CCMR2_OC3PE_Pos) << ((channel - 2)*8));
		_instance->CCMR2 |= ((conf->fastmode << TIM_CCMR2_OC3FE_Pos) << ((channel - 2)*8));
		_instance->CCMR2 |= ((conf->invert << TIM_CCMR2_OC3M_Pos) << ((channel - 2)*8));
	}

	return ret;
}

inline void  tim::pwm_output_set_duty(tim_channel_t channel, uint32_t pwm){
	switch(channel){
		case TIM_CHANNEL1:
			_instance->CCR1 = pwm;
		break;
		case TIM_CHANNEL2:
			_instance->CCR2 = pwm;
		break;
		case TIM_CHANNEL3:
			_instance->CCR3 = pwm;
		break;
		case TIM_CHANNEL4:
			_instance->CCR4 = pwm;
		break;
		default:
		break;
	};
}

inline void tim::pwm_output_start(tim_channel_t channel, uint32_t pwm){
	_instance->CR1 &=~ TIM_CR1_CEN;
	pwm_output_set_duty(channel, pwm);
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));
	_instance->CR1 |= TIM_CR1_CEN;
}

inline void tim::pwm_output_stop(tim_channel_t channel){
	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->CR1 &=~ TIM_CR1_CEN;
}


inline void tim::pwm_output_start_it(tim_channel_t channel, uint32_t pwm){
	pwm_output_set_duty(channel, pwm);

	_instance->DIER |= (TIM_DIER_CC1IE << channel);
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));
	enable_interrupt();
	_instance->CR1 |= TIM_CR1_CEN;
}

inline void tim::pwm_output_stop_it(tim_channel_t channel){
	_instance->DIER &=~ (TIM_DIER_CC1IE << channel);
	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->CR1 &=~ TIM_CR1_CEN;
	disable_interrupt();
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::pwm_output_start_dmac(tim_channel_t channel, uint32_t *ppwm_buffer, uint16_t size){
	err_t ret;
	dmac_t dmac = NULL;
	uint32_t CCRx_addr = 0x00U;

	ASSERT_THEN_RETURN_VALUE((ppwm_buffer == NULL || size == 0U),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "Invalid parameter");

	switch(channel){
		case TIM_CHANNEL1:
			dmac = _dmac_ch1;
			CCRx_addr = (uint32_t)&_instance->CCR1;
		break;
		case TIM_CHANNEL2:
			dmac = _dmac_ch2;
			CCRx_addr = (uint32_t)&_instance->CCR2;
		break;
		case TIM_CHANNEL3:
			dmac = _dmac_ch3;
			CCRx_addr = (uint32_t)&_instance->CCR3;
		break;
		case TIM_CHANNEL4:
			dmac = _dmac_ch4;
			CCRx_addr = (uint32_t)&_instance->CCR4;
		break;
	};
	ASSERT_THEN_RETURN_VALUE((dmac == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "TIM DMA invalid");

	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->DIER &=~ TIM_DIER_CC1DE << channel;
	_instance->CR1  &=~ TIM_CR1_CEN;

	struct dmac_session_config_t xfer_conf = {
		.psource = (uint32_t)ppwm_buffer,
		.pdest = (uint32_t)CCRx_addr,
		.xfersize = size,
	};
	ret = dmac->config_start_session(xfer_conf);
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		TIM_DBG("TIM DMA start error");
		dmac->stop_session();
		return ret;
	}

	_instance->DIER |= TIM_DIER_CC1DE << channel;
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));
	_instance->CR1  |= TIM_CR1_CEN;

	return ret;
}

err_t tim::pwm_output_stop_dmac(tim_channel_t channel){
	err_t ret;
	dmac_t dmac = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dmac = _dmac_ch1;
		break;
		case TIM_CHANNEL2:
			dmac = _dmac_ch2;
		break;
		case TIM_CHANNEL3:
			dmac = _dmac_ch3;
		break;
		case TIM_CHANNEL4:
			dmac = _dmac_ch4;
		break;
	};
	ASSERT_THEN_RETURN_VALUE((dmac == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "TIM DMA invalid");

	_instance->DIER &=~ (TIM_DIER_CC1DE << channel);
	dmac->stop_session();;
	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->CR1  &=~ TIM_CR1_CEN;

	return ret;
}
#endif



/**
 * Timer PWM input function.
 */
err_t tim::pwm_input_initialize(tim_channel_t channel, tim_config_pwm_t *conf){
	err_t ret;
	gpio_pin_t pwmpin;

	ASSERT_THEN_RETURN_VALUE((channel == TIM_CHANNEL3 || channel == TIM_CHANNEL4),
				ERROR_SET(ret, E_NOT_SUPPORTED), ret, LOG_ERROR, TAG, "TIM Channel 3 and Channel 4 not supported this mode");
	ASSERT_THEN_RETURN_VALUE((conf->polarity == TIM_BOTH_EDGE),
					ERROR_SET(ret, E_NOT_SUPPORTED), ret, LOG_ERROR, TAG, "TIM PWM input not supported both edge");

	if(gpio_str_decode(conf->pin, &pwmpin) == false) return {E_INVALID, CODE_LINE};

	gpio_port_clock_enable(pwmpin.port);
#if defined(STM32F1)
	gpio_set_direction(pwmpin.port, pwmpin.pinnum, GPIO_INPUT);
#elif defined(STM32F4)
	gpio_set_direction(pwmpin.port, pwmpin.pinnum, GPIO_FUNCTION);
	gpio_set_function(pwmpin.port, pwmpin.pinnum, _gpio_func);
	gpio_set_type(pwmpin.port, pwmpin.pinnum, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	_instance->CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

	_instance->SMCR &=~ TIM_SMCR_TS_Msk;
	_instance->SMCR |= ((channel+5U) << TIM_SMCR_TS_Pos) | TIM_SMCR_SMS_2;

	_instance->CCMR1 &=~ (TIM_CCMR1_IC1F_Msk << (channel*8));
	_instance->CCMR1 |= (conf->ch1_filter << TIM_CCMR1_IC1F_Pos);
	_instance->CCMR1 |= (conf->ch2_filter << TIM_CCMR1_IC2F_Pos);

	_instance->CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	if(channel == TIM_CHANNEL1){
		_instance->CCER |= (conf->polarity << TIM_CCER_CC1P_Pos);
		_instance->CCER |= (!(conf->polarity) << TIM_CCER_CC2P_Pos);
	}
	else{
		_instance->CCER |= (!(conf->polarity) << TIM_CCER_CC1P_Pos);
		_instance->CCER |= (conf->polarity << TIM_CCER_CC2P_Pos);
	}

	_instance->CCMR1 &=~ (TIM_CCMR1_CC1S_Msk << (channel*8U));
	_instance->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;

	return ret;
}

err_t tim::pwm_input_start(tim_channel_t channel){
	return inputcapture_start(channel);
}

err_t tim::pwm_input_stop(tim_channel_t channel){
	return inputcapture_stop(channel);
}

err_t tim::pwm_input_start_it(tim_channel_t channel){
	return inputcapture_start_it(channel);
}

err_t tim::pwm_input_stop_it(tim_channel_t channel){
	return inputcapture_stop_it(channel);
}
#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::pwm_input_start_dmac(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size){
	return inputcapture_start_dmac(channel, pwm_buffer, size);
}

err_t tim::pwm_input_stop_dmac(tim_channel_t channel){
	return inputcapture_stop_dmac(channel);
}
#endif




/* TIMER ENCODER MODE */
err_t tim::encoder_initialize(tim_config_encoder_t *conf){
	err_t ret;
	gpio_pin_t encApin, encBpin;

	if(gpio_str_decode(conf->encA_ch1_pin, &encApin) == false) return {E_INVALID, CODE_LINE};
	if(gpio_str_decode(conf->encB_ch2_pin, &encBpin) == false) return {E_INVALID, CODE_LINE};
	gpio_port_clock_enable(encApin.port);
	gpio_port_clock_enable(encBpin.port);
#if defined(STM32F1)
	gpio_config_t gpioconf = {
		.port = encApin.port,
		.pinnum = encApin.pinnum,
		.direction = GPIO_INPUT,
		.pullresistor = GPIO_PULL_UP,
	};
	gpio_init(&gpioconf);
	gpioconf.port = encBpin.port;
	gpioconf.pinnum = encBpin.pinnum;
	gpio_init(&gpioconf);
#elif defined(STM32F4)
	gpio_set_direction(encApin.port, encApin.pinnum, GPIO_FUNCTION);
	gpio_set_direction(encBpin.port, encBpin.pinnum, GPIO_FUNCTION);
	gpio_set_function(encApin.port, encApin.pinnum, _gpio_func);
	gpio_set_function(encBpin.port, encBpin.pinnum, _gpio_func);
	gpio_set_type(encApin.port, encApin.pinnum, GPIO_OUTPUT_PUSHPULL);
	gpio_set_type(encBpin.port, encBpin.pinnum, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	if	   (_instance == TIM1)  _IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_instance == TIM8)  _IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */
	_instance->SMCR &=~ TIM_SMCR_SMS;
	_instance->SMCR |= (conf->mode << TIM_SMCR_SMS_Pos);

	_instance->CCMR1 = 0U;
	_instance->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	_instance->CCMR1 |= ((conf->encA_ch1_prescaler << TIM_CCMR1_IC1PSC_Pos) | (conf->encB_ch2_prescaler << TIM_CCMR1_IC2PSC_Pos));
	_instance->CCMR1 |= ((conf->encA_ch1_filter << TIM_CCMR1_IC1F_Pos) | (conf->encB_ch2_filter << TIM_CCMR1_IC2F_Pos));

	_instance->CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	_instance->CCER |= ((conf->encA_ch1_edge << TIM_CCER_CC1P_Pos) | (conf->encB_ch2_edge << TIM_CCER_CC2P_Pos));

	return ret;
}

err_t tim::tim_encoder_start(void){
	err_t ret;

	if(_instance->CR1 & TIM_CR1_CEN){
		set_status(&ret, STT_BUSY, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_instance->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_instance->CCER |= TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_instance->CCER |= TIM_CCER_CC2E;
	}
	else{
		_instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	}

	_instance->CR1 |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_encoder_stop(void){
	err_t ret;

	if(!(_instance->CR1 & TIM_CR1_CEN)){
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_instance->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_instance->CCER &=~ TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_instance->CCER &=~ TIM_CCER_CC2E;
	}
	else{
		_instance->CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
	}

	_instance->CR1 &=~ TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_encoder_start_it(void){
	err_t ret;

	if(_conf->interrupt == TIM_INTERRUPT_DISABLE){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s->Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}


	if(_conf->interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY){
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s->Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
#if CONFIG_SYS_CONFIG_FAIL_RESET_ENABLE
#if CONFIG_PERIPH_TIM_LOG
		LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_SYS_CONFIG_FAIL_RESET_TIME);
#endif /* CONFIG_PERIPH_TIM_LOG */
		systick_delay_ms(CONFIG_SYS_CONFIG_FAIL_RESET_TIME*1000U);
		__NVIC_SystemReset();
#endif /* CONFIG_SYS_CONFIG_FAIL_RESET_ENABLE */
		return ret;
	}


	if(_instance->CR1 & TIM_CR1_CEN){
		set_status(&ret, STT_BUSY, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_instance->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_instance->CCER |= TIM_CCER_CC1E;
		_instance->DIER |= TIM_DIER_CC1IE;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_instance->CCER |= TIM_CCER_CC2E;
		_instance->DIER |= TIM_DIER_CC2IE;
	}
	else{
		_instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
		_instance->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
	}

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	__NVIC_EnableIRQ(_IRQn);
	__NVIC_ClearPendingIRQ(_IRQn);

	_instance->CR1 |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_encoder_stop_it(void){
	err_t ret;

	if(_conf->interrupt == TIM_INTERRUPT_DISABLE){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s->Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	if(!(_instance->CR1 & TIM_CR1_CEN)){
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->CR1 &=~ TIM_CR1_CEN;
	_instance->DIER &=~ TIM_DIER_UIE;
	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);

	return ret;
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::tim_encoder_start_dmac(uint32_t *encA_buffer, uint32_t *encB_buffer, uint16_t size){
	err_t ret;

	if((_dmac_ch1 == NULL && _dmac_ch2 == NULL) || (encA_buffer == NULL && encB_buffer == NULL)){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer not set dmac or data buffer, can't start in dmac mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	_instance->CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_instance->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	switch(mode){
		case TIM_ENCODER_MODE1:
			_instance->DIER &=~ TIM_DIER_CC1DE;
			_instance->CCER &=~ TIM_CCER_CC1E;

			CCRx_addr = (uint32_t)&_instance->CCR1;
			ret = _dmac_ch1-> dmac_start_transfer(CCRx_addr, (uint32_t)encA_buffer, size);
			if(status_have_error(&ret)) {
				set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
				LOG_ERROR(TAG, "%s->%s, Timer dmac start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
				return ret;
			}

			_instance->DIER |= TIM_DIER_CC1DE;
			_instance->CCER |= TIM_CCER_CC1E;
		break;

		case TIM_ENCODER_MODE2:
			_instance->DIER &=~ TIM_DIER_CC2DE;
			_instance->CCER &=~ TIM_CCER_CC2E;

			CCRx_addr = (uint32_t)&_instance->CCR2;
			ret = _dmac_ch2-> dmac_start_transfer(CCRx_addr, (uint32_t)encB_buffer, size);
			if(status_have_error(&ret)) {
				set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
				LOG_ERROR(TAG, "%s->%s, Timer dmac start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
				return ret;
			}

			_instance->DIER |= TIM_DIER_CC2DE;
			_instance->CCER |= TIM_CCER_CC2E;
		break;

		case TIM_ENCODER_MODE3:
			_instance->DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_instance->CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

			CCRx_addr = (uint32_t)&_instance->CCR1;
			ret = _dmac_ch1-> dmac_start_transfer(CCRx_addr, (uint32_t)encA_buffer, size);
			if(status_have_error(&ret)) {
				set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
				LOG_ERROR(TAG, "%s->%s, Timer dmac start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
				return ret;
			}

			CCRx_addr = (uint32_t)&_instance->CCR2;
			ret = _dmac_ch2-> dmac_start_transfer(CCRx_addr, (uint32_t)encB_buffer, size);
			if(status_have_error(&ret)) {
				set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
				LOG_ERROR(TAG, "%s->%s, Timer dmac start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
				return ret;
			}

			_instance->DIER |= (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
		break;

		default:
		break;
	};
	_instance->CR1  |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_encoder_stop_dmac(void){
	err_t ret;

	if((_dmac_ch1 == NULL && _dmac_ch2 == NULL)){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer not set dmac or data buffer, can't stop in dmac mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	if(((_instance->DIER & TIM_DIER_CC1DE) || (_instance->DIER & TIM_DIER_CC2DE)) && (_instance->CR1 & TIM_CR1_CEN)){
		tim_encoder_mode_t mode = (tim_encoder_mode_t)((_instance->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
		switch(mode){
			case TIM_ENCODER_MODE1:
				_instance->DIER &=~ TIM_DIER_CC1DE;
				ret = _dmac_ch1-> dmac_stop_transfer();;
				if(status_have_error(&ret)) {
					set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
					LOG_ERROR(TAG, "%s->%s, Timer dmac stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
					return ret;
				}
				_instance->CCER &=~ TIM_CCER_CC1E;
			break;

			case TIM_ENCODER_MODE2:
				_instance->DIER &=~ TIM_DIER_CC2DE;
				ret = _dmac_ch2-> dmac_stop_transfer();;
				if(status_have_error(&ret)) {
					set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
					LOG_ERROR(TAG, "%s->%s, Timer dmac stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
					return ret;
				}
				_instance->CCER &=~ TIM_CCER_CC2E;
			break;

			case TIM_ENCODER_MODE3:
				_instance->DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
				ret = _dmac_ch1-> dmac_stop_transfer();;
				if(status_have_error(&ret)) {
					set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
					LOG_ERROR(TAG, "%s->%s, Timer dmac stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
					return ret;
				}
				ret = _dmac_ch2-> dmac_stop_transfer();;
				if(status_have_error(&ret)) {
					set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
					LOG_ERROR(TAG, "%s->%s, Timer dmac stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
					return ret;
				}
				_instance->CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
			break;

			default:
			break;
		};
		_instance->CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer dmac not started.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
	}

	return ret;
}
#endif

int16_t tim::tim_encoder_get_counter(void){
	return (int16_t)((int16_t)_counter/4);
}

uint32_t tim::tim_encoder_get_base_counter(void){
	return _counter;
}


/* TIMER INPUT CAPTURE MODE */
err_t tim::tim_set_mode_inputcapture(tim_channel_t channel, tim_inputcapture_t *conf){
	err_t ret;

	gpio_port_clock_enable(conf->port);
#if defined(STM32F1)
	gpio_set_mode(conf->port, conf->pin, GPIO_INPUT);
#elif defined(STM32F4)
	gpio_af_t func;
	if(_instance == TIM1 || _instance == TIM2) 										func = AF1_TIM1_2;
	else if(_instance == TIM3 || _instance == TIM4 || _instance == TIM5) 					func = AF2_TIM3_5;
	else if(_instance == TIM8 || _instance == TIM9 || _instance == TIM10 || _instance == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_af(conf->port, conf->pin, func);
	gpio_set_af_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	if	   (_instance == TIM1)  _IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_instance == TIM8)  _IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */

	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4U));
	if(channel < TIM_CHANNEL3){
		_instance->CCMR1 &=~ (0xFF << (channel));
		_instance->CCMR1 |= (TIM_CCMR1_CC1S_0 << (channel*8U));

		_instance->CCMR1 |= ((conf->filter << TIM_CCMR1_IC1F_Pos) << (channel*8U));

		_instance->CCMR1 |= (conf->prescaler << (channel*8U));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){
		_instance->CCMR2 &=~ (0xFF << ((channel-2)));
		_instance->CCMR2 |= (TIM_CCMR2_CC3S_0 << ((channel-2)*8U));

		_instance->CCMR2 |= ((conf->filter << TIM_CCMR2_IC3F_Pos) << ((channel-2)*8U));

		_instance->CCMR2 |=(conf->prescaler << (channel*8U));
	}
	_instance->CCER &=~ ((TIM_CCER_CC1P | TIM_CCER_CC1NP) << (channel*4));
	_instance->CCER |= ((conf->polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

err_t tim::tim_inputcapture_start(tim_channel_t channel){
	err_t ret;

	if(_instance->CR1 & TIM_CR1_CEN){
		set_status(&ret, STT_BUSY, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));
	_instance->CR1 |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_inputcapture_stop(tim_channel_t channel){
	err_t ret;

	if(!(_instance->CR1 & TIM_CR1_CEN)){
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->CR1 &=~ TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_inputcapture_start_it(tim_channel_t channel){
	err_t ret;

	if(_instance->CR1 & TIM_CR1_CEN){
		set_status(&ret, STT_BUSY, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->DIER |= (TIM_DIER_CC1IE << channel);
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	__NVIC_EnableIRQ(_IRQn);
	__NVIC_ClearPendingIRQ(_IRQn);

	_instance->CR1 |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_inputcapture_stop_it(tim_channel_t channel){
	err_t ret;

	if(!(_instance->CR1 & TIM_CR1_CEN)){
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_WARN(TAG, "%s->%s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->DIER &=~ (TIM_DIER_CC1IE << channel);
	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);

	_instance->CR1 &=~ TIM_CR1_CEN;

	return ret;
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::tim_inputcapture_start_dmac(tim_channel_t channel, uint32_t *capture_buffer, uint16_t size){
	err_t ret;
	dmac_t dmac = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dmac = _dmac_ch1;
		break;
		case TIM_CHANNEL2:
			dmac = _dmac_ch2;
		break;
		case TIM_CHANNEL3:
			dmac = _dmac_ch3;
		break;
		case TIM_CHANNEL4:
			dmac = _dmac_ch4;
		break;
		default:
		break;
	};

	if(dmac == NULL){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer not set dmac, can't start in dmac mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_instance->DIER &=~ TIM_DIER_CC1DE << channel;
	_instance->CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_instance->CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_instance->CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_instance->CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_instance->CCR4;
		break;
		default:
		break;
	};
	ret = dmac-> dmac_start_transfer(CCRx_addr, (uint32_t)capture_buffer, size);
	if(status_have_error(&ret)) {
		set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer dmac start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}
	_instance->DIER |= TIM_DIER_CC1DE << channel;
	_instance->CCER |= (TIM_CCER_CC1E << (channel*4));
	_instance->CR1  |= TIM_CR1_CEN;

	return ret;
}

err_t tim::tim_inputcapture_stop_dmac(tim_channel_t channel){
	err_t ret;
	dmac_t dmac = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dmac = _dmac_ch1;
		break;
		case TIM_CHANNEL2:
			dmac = _dmac_ch2;
		break;
		case TIM_CHANNEL3:
			dmac = _dmac_ch3;
		break;
		case TIM_CHANNEL4:
			dmac = _dmac_ch4;
		break;
		default:
		break;
	};

	if(dmac == NULL){
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer not set dmac, can't stop in dmac mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
		return ret;
	}

	if((_instance->DIER & TIM_DIER_CC1DE << channel) && (_instance->CR1 & TIM_CR1_CEN)){
		_instance->DIER &=~ (TIM_DIER_CC1DE << channel);
		ret = dmac-> dmac_stop_transfer();;
		if(status_have_error(&ret)) {
			set_status_line(&ret, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
			LOG_ERROR(TAG, "%s->%s, Timer dmac stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
			return ret;
		}
		_instance->CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_instance->CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_status(&ret, STT_FAIL, __LINE__);
#if CONFIG_PERIPH_TIM_LOG
		LOG_ERROR(TAG, "%s->%s, Timer dmac not started.", __FILE__, __FUNCTION__);
#endif /* CONFIG_PERIPH_TIM_LOG */
	}

	return ret;
}
#endif

uint32_t tim::tim_get_capture_counter(tim_channel_t channel){
	switch(channel){
		case TIM_CHANNEL1:
			return _instance->CCR1;

		case TIM_CHANNEL2:
			return _instance->CCR2;

		case TIM_CHANNEL3:
			return _instance->CCR3;

		case TIM_CHANNEL4:
			return _instance->CCR4;
		default:
			return 0U;
	};
	return 0U;
}

/* TIMER OuTPUT COMPARE MODE*/
err_t tim::tim_set_mode_outputcompare(tim_channel_t channel, tim_outputcompare_t *conf){
	err_t ret;

	gpio_port_clock_enable(conf->port);
#if defined(STM32F1)
	gpio_set_af(conf->port, conf->pin, GPIO_ALTERNATE_PUSHPULL);
#elif defined(STM32F4)
	gpio_af_t func;
	if(_instance == TIM1 || _instance == TIM2) 										func = AF1_TIM1_2;
	else if(_instance == TIM3 || _instance == TIM4 || _instance == TIM5) 					func = AF2_TIM3_5;
	else if(_instance == TIM8 || _instance == TIM9 || _instance == TIM10 || _instance == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_instance == TIM12 || _instance == TIM13 || _instance == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_status(&ret, STT_ERR_NOT_SUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_af(conf->port, conf->pin, func);
	gpio_set_af_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */

	if	   (_instance == TIM1)  _IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_instance == TIM8)  _IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */

	_instance->CCER &=~ TIM_CCER_CC1E;

	if(channel < TIM_CHANNEL3){
		_instance->CCMR1 &=~ (0xFF << (channel*8));
		_instance->CCMR1 |= ((conf->mode << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if (channel > TIM_CHANNEL2 && channel < TIM_NOCHANNEL){
		_instance->CCMR2 &=~ (0xFF << ((channel-2)*8));
		_instance->CCMR2 |= ((conf->mode << TIM_CCMR2_OC3M_Pos) << ((channel-2)*8));
	}
	_instance->CCER &=~ (0x0F << (channel*4));
	_instance->CCER |= ((conf->level_polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

err_t tim::tim_outputcompare_start(tim_channel_t channel, uint32_t value){
	return tim_pwm_output_start(channel, value);
}

err_t tim::tim_outputcompare_stop(tim_channel_t channel){
	return tim_pwm_output_stop(channel);
}

err_t tim::tim_outputcompare_start_it(tim_channel_t channel, uint32_t value){
	return tim_pwm_output_start_it(channel, value);
}

err_t tim::tim_outputcompare_stop_it(tim_channel_t channel){
	return tim_pwm_output_stop_it(channel);
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t tim::tim_outputcompare_start_dmac(tim_channel_t channel, uint32_t *oc_buffer, uint16_t size){
	return tim_pwm_output_start_dmac(channel, oc_buffer, size);
}

err_t tim::tim_outputcompare_stop_dmac(tim_channel_t channel){
	return tim_pwm_output_stop_dmac(channel);
}
#endif


err_t tim::tim_set_pulse(tim_channel_t channel, uint32_t pulse){
	err_t ret;


	return ret;
}


static inline void TIMCommon_IRQHandler(tim *ptim){
	tim_event_t event = TIM_EVENT_NOEVENT;
	tim_channel_t channel = TIM_NOCHANNEL;

	ptim->_counter = ptim->_instance->CNT;

	/* TIMER CAPTURE-COMPARE 1 INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_CC1IF && ptim->_instance->DIER & TIM_DIER_CC1IE){
		ptim->_instance->SR =~ TIM_SR_CC1IF;
		event = TIM_EVENT_CAPTURECOMPARE1;
		channel = TIM_CHANNEL1;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 2 INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_CC2IF && ptim->_instance->DIER & TIM_DIER_CC2IE){
		ptim->_instance->SR =~ TIM_SR_CC2IF;
		event = TIM_EVENT_CAPTURECOMPARE2;
		channel = TIM_CHANNEL2;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 3 INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_CC3IF && ptim->_instance->DIER & TIM_DIER_CC3IE){
		ptim->_instance->SR =~ TIM_SR_CC3IF;
		event = TIM_EVENT_CAPTURECOMPARE3;
		channel = TIM_CHANNEL3;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 4 INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_CC4IF && ptim->_instance->DIER & TIM_DIER_CC4IE){
		ptim->_instance->SR =~ TIM_SR_CC4IF;
		event = TIM_EVENT_CAPTURECOMPARE4;
		channel = TIM_CHANNEL4;
		goto EventCB;
	}

	/* TIMER UPDATE INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_UIF && ptim->_instance->DIER & TIM_DIER_UIE){
		ptim->_instance->SR =~ TIM_SR_UIF;
		event = TIM_EVENT_UPDATE;
		goto EventCB;
	}

	/* TIMER BREAK INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_BIF && ptim->_instance->DIER & TIM_DIER_BIE){
		ptim->_instance->SR =~ TIM_SR_BIF;
		event = TIM_EVENT_BREAK;
		goto EventCB;
	}

	/* TIMER TRIGER INTERRUPT */
	if(ptim->_instance->SR & TIM_SR_TIF && ptim->_instance->DIER & TIM_DIER_TIE){
		ptim->_instance->SR =~ TIM_SR_TIF;
		event = TIM_EVENT_TRIGER;
		goto EventCB;
	}

	EventCB:
	if(ptim->_event_handler != NULL) ptim->_event_handler(channel, event, ptim->_event_parameter);
}

#if defined(TIM1)
tim tim_1(TIM1);
tim_t tim1 = &tim_1;
void TIM1_CC_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_1);
}
#if !defined(TIM9)
void TIM1_BRK_TIM9_IRQHandler(void){
#if USE_TIM1
//	TIMCommon_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM9) */
#if !defined(TIM10)
void TIM1_UP_TIM10_IRQHandler(void){
#if USE_TIM1
//	TIMCommon_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM10) */
#if !defined(TIM11)
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#if USE_TIM1
//	TIMCommon_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM11) */
#endif /* defined(TIM1) */

#if defined(TIM2)
tim tim_2(TIM2);
tim_t tim2 = &tim_2;
void TIM2_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_2);
}
#endif /* defined(TIM2) */

#if defined(TIM3)
tim tim_3(TIM3);
tim_t tim3 = &tim_3;
void TIM3_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_3);
}
#endif /* defined(TIM3) */

#if defined(TIM4)
tim tim_4(TIM4);
tim_t tim4 = &tim_4;
void TIM4_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_4);
}
#endif /* defined(TIM4) */

#if defined(TIM5)
tim tim_5(TIM5);
tim_t tim5 = &tim_5;
void TIM5_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_5);
}
#endif /* defined(TIM5) */

#if defined(TIM6)
tim tim_6(TIM6);
tim_t tim6 = &tim_6;
void TIM6_DAC_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_6);
#if ENABLE_DAC
	DAC_IRQHandler();
#endif
}
#endif /* defined(TIM6) */

#if defined(TIM7)
tim tim_7(TIM7);
tim_t tim7 = &tim_7;
void TIM7_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_7);
}
#endif /* defined(TIM7) */

#if defined(TIM8)
tim tim_8(TIM8);
tim_t tim8 = &tim_8;
void TIM8_CC_IRQHandler(void){
	TIMCommon_IRQHandler(&tim_8);
}
#if !defined(TIM12)
void TIM8_BRK_TIM12_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
}
#endif /* !defined(TIM12) */
#if !defined(TIM13)
void TIM8_UP_TIM13_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
}
#endif /* !defined(TIM13) */
#if !defined(TIM14)
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
	TIMCommon_IRQHandler(&tim_14);
}
#endif /* !defined(TIM14) */
#endif /* defined(TIM8) */

#if defined(TIM9) && defined(TIM1)
tim tim_9(TIM9);
tim_t tim9 = &tim_9;
void TIM1_BRK_TIM9_IRQHandler(void){
#if USE_TIM1
	TIMCommon_IRQHandler(&tim_1);
#endif
	TIMCommon_IRQHandler(&tim_9);
}
#endif /* defined(TIM9) && defined(TIM1) */

#if defined(TIM10) && defined(TIM1)
tim tim_10(TIM10);
tim_t tim10 = &tim_10;
void TIM1_UP_TIM10_IRQHandler(void){
#if USE_TIM1
	TIMCommon_IRQHandler(&tim_1);
#endif
	TIMCommon_IRQHandler(&tim_10);
}
#endif /* defined(TIM10) && defined(TIM1) */

#if defined(TIM11) && defined(TIM1)
tim tim_11(TIM11);
tim_t tim11 = &tim_11;
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#if USE_TIM1
	TIMCommon_IRQHandler(&tim_1);
#endif
	TIMCommon_IRQHandler(&tim_11);
}
#endif /* defined(TIM11) && defined(TIM1) */

#if defined(TIM12) && defined(TIM8)
tim tim_12(TIM12);
tim_t tim12 = &tim_12;
void TIM8_BRK_TIM12_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
	TIMCommon_IRQHandler(&tim_12);
}
#endif /* defined(TIM12) && defined(TIM8) */

#if defined(TIM13) && defined(TIM8)
tim tim_13(TIM13);
tim_t tim13 = &tim_13;
void TIM8_UP_TIM13_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
	TIMCommon_IRQHandler(&tim_13);
}
#endif /* defined(TIM13) && defined(TIM8) */

#if defined(TIM14) && defined(TIM8)
tim tim_14(TIM14);
tim_t tim14 = &tim_14;
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#if USE_TIM8
	TIMCommon_IRQHandler(&tim_8);
#endif
	TIMCommon_IRQHandler(&tim_14);
}
#endif /* defined(TIM14) && defined(TIM8) */


#endif



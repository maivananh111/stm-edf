/*
 * adc.cpp
 *
 *  Created on: Jun 30, 2022
 *      Author: A315-56
 */

#include "drivers/adc.h"
#if PERIPHERAL_ADC_AVAILABLE

#include "drivers/gpio.h"
#include "drivers/rcc.h"
#include "drivers/system.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"

#if CONFIG_PERIPH_UART_LOG
#include "common/log_monitor.h"
#endif /* CONFIG_PERIPH_UART_LOG */



static const char *_TAG = "ADC";


#ifdef CONFIG_PERIPH_ADC_LOG
#define ADC_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, _TAG, __MSG__);
#else
#define ADC_DBG(__MSG__)\
{}
#endif

static void ADCCommon_IRQHandler(adc *adc);

adc_t adc_str_decode(char *str){
#if defined(SPI1)
	if(strcasecmp(str, "ADC1") == 0)
		return adc1;
#endif /* defined(SPI1) */
#if defined(SPI2)
	if(strcasecmp(str, "ADC2") == 0)
		return adc2;
#endif /* defined(SPI2) */
#if defined(SPI3)
	if(strcasecmp(str, "ADC3") == 0)
		return adc3;
#endif /* defined(SPI3) */
	return adc1;
}

adc::adc(ADC_TypeDef *adc){
	_instance = adc;
}


err_t adc::initialize(adc_config_t *conf){
	err_t ret;

	_conf = conf;
	uint32_t tmp_ADC_SMPR1 = 0;
	uint32_t tmp_ADC_SMPR2 = 0;

	/* ADC CLOCK ENABLE */
	if(_instance == ADC1) RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;

#if defined(ADC2)
	if(_instance == ADC2) RCC -> APB2ENR |= RCC_APB2ENR_ADC2EN;
#endif /* defined(ADC2) */
#if defined(ADC3)
	if(_instance == ADC3) RCC -> APB2ENR |= RCC_APB2ENR_ADC3EN;
#endif /* defined(ADC3) */

	/* GPIO SETUP AT ANALOG MODE */
	for(uint8_t i=0; i<_conf -> num_channel; i++) {
		if(_conf -> pin_list[i] <= 15) gpio_set_direction(_conf -> port_list[i], _conf -> pin_list[i], GPIO_ANALOG);
	}

	/* ADC PRESCALER DIVISION */
#if defined(STM32F1)
	RCC -> CFGR |= (_conf->prescaler << RCC_CFGR_ADCPRE_Pos);
#elif defined(STM32F4)
	ADC123_COMMON -> CCR &=~ ADC_CCR_ADCPRE;
	ADC123_COMMON -> CCR |= _conf -> prescaler;
#endif

	/* ADC CONFIGURATION */
	_instance -> CR1 &=~ ADC_CR1_SCAN;
	if(sizeof(_conf -> pin_list) > 1)
		_instance -> CR1 |= ADC_CR1_SCAN;   // ADC SCAN MODE.

#if defined(STM32F4)
	_instance -> CR1 &=~ ADC_CR1_RES;
	_instance -> CR1 |= _conf -> resolution; // ADC RESOLUTION.
#endif /* STM32F4 */

	_instance -> CR2 &=~ ADC_CR2_ALIGN; // ADC DATA ALIGN RIGHT.
	if(_conf -> dataalign == ADC_DATAALIGN_LEFT)
		_instance -> CR2 |= ADC_CR2_ALIGN;

#if defined(STM32F1)
	_instance -> CR2 |= ADC_CR2_EXTSEL; // ADC START BY SWSTART
#elif defined(STM32F4)
	_instance -> CR2 &=~ ADC_CR2_EXTSEL; // ADC SELECT SOFTWARE START.
	_instance -> CR2 &=~ ADC_CR2_EXTEN;  // ADC DISABLE EXTERNAL TRIGER START.
#endif /* STM32F4 */

	_instance -> CR2 &=~ ADC_CR2_CONT;
	_instance -> CR2 |= _conf -> continuos; // ADC CONTINUOUS MODE ENABLE.
	_instance -> CR1 &=~ ADC_CR1_DISCEN;    // ADC DISCONTINUOUS MODE DISABLE.

	for(uint8_t i=0; i<_conf -> num_channel; i++){
		if(_conf -> rank_list[i] <= 9) tmp_ADC_SMPR2 |= (_conf -> sampletime_list[i] << (3*_conf -> rank_list[i]));      // SMPR1 REGISTER
		else 				           tmp_ADC_SMPR1 |= (_conf -> sampletime_list[i] << (3*(_conf -> rank_list[i] - 10))); // SMPR1 REGISTER
	}
	_instance -> SMPR1 = tmp_ADC_SMPR1; // ADC CHANNEL 10-17 SAMPLING TIME
	_instance -> SMPR2 = tmp_ADC_SMPR2; // ADC CHANNEL 0 - 9 SAMPLING TIME

	uint8_t num_reg = ceil((float)_conf -> num_channel/6.0);
	uint32_t ADC_SQR1 = 0, ADC_SQR2 = 0, ADC_SQR3 = 0;
	if(num_reg == 0) num_reg++;
	if(num_reg == 1 && _conf -> num_channel <= 6){
		for(uint8_t i=0; i<_conf -> num_channel; i++)  ADC_SQR3 |= (_conf -> rank_list[i] << i*5);
	}
	else if(num_reg == 2 && _conf -> num_channel <=12){
		for(uint8_t i=0; i<6; i++)                          ADC_SQR3 |= (_conf -> rank_list[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<_conf -> num_channel; i++)  ADC_SQR2 |= (_conf -> rank_list[i] << (i-6)*5); // 6 -> numcvt
	}
	else{
		for(uint8_t i=0; i<6; i++)                          ADC_SQR3 |= (_conf -> rank_list[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<12; i++)                         ADC_SQR2 |= (_conf -> rank_list[i] << (i-6)*5); // 6 -> 11
		for(uint8_t i=12; i<_conf -> num_channel; i++) ADC_SQR1 |= (_conf -> rank_list[i] << (i-12)*5); // 12 -> numcvt
	}
	_instance -> SQR1 = ADC_SQR1; // ADC SEQUENCE NUMBER.
	_instance -> SQR2 = ADC_SQR2; // ADC SEQUENCE NUMBER.
	_instance -> SQR3 = ADC_SQR3; // ADC SEQUENCE NUMBER.

	_instance -> SQR1 |= ((_conf -> num_channel - (uint8_t)1) << ADC_SQR1_L_Pos); // ADC NUMBER OF CONVERSION SEQUENCE

	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
		ADC_DBG("Interrupt priority out of range");
		system_reset();
		return ret;
	}

	_IRQn = ADC_IRQn;
	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	_mutex = xSemaphoreCreateMutex();

	return ret;
}

err_t adc::deinitialize(void){
#if PERIPHERAL_DMAC_AVAILABLE
	_dma = NULL;
#endif /* PERIPHERAL_DMAC_AVAILABLE */

	CLEAR_REG(_instance->CR1);
	CLEAR_REG(_instance->CR2);
	CLEAR_REG(_instance->SMPR1);
	CLEAR_REG(_instance->SMPR2);
	CLEAR_REG(_instance->SQR1);
	CLEAR_REG(_instance->SQR2);
	CLEAR_REG(_instance->SQR3);
	__IO uint32_t tmp = READ_REG(_instance->DR);
	CLEAR_REG(_instance->SR);
	(void)tmp;

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);

	for(uint8_t i=0; i<_conf -> num_channel; i++) {
		if(_conf -> pin_list[i] <= 15) gpio_deinit(_conf -> port_list[i], _conf -> pin_list[i]);
	}

    xSemaphoreTake(_mutex, portMAX_DELAY);
    vSemaphoreDelete(_mutex);

	return {};
}


#if PERIPHERAL_DMAC_AVAILABLE
void adc::link_dmac(dmac_t dma){
	_dma = dma;

	if(_dmaconf != NULL) free(_dmaconf);
	_dmaconf = (dmac_config_t *)malloc(sizeof(dmac_config_t));
	if(_dmaconf != NULL){
		_dmaconf->direction = DMAC_MEM_TO_PERIPH;
		_dmaconf->datasize  = DMAC_DATASIZE_8BIT;
		_dmaconf->interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT;

		_dma->initialize(_dmaconf);
		_dma->register_event_handler(FUNC_BIND(&adc::dmac_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
}

void adc::unlink_dmac(void){
	if(_dmaconf != NULL){
		_dma->deinitialize();
		_dma->unregister_event_handler();
	}
	_dma = NULL;
	if(_dmaconf != NULL) free(_dmaconf);
	_dmaconf = NULL;
}
#endif /* PERIPHERAL_DMAC_AVAILABLE */


void adc::start_convert(void){
	_instance -> CR2 |= ADC_CR2_ADON; // ENABLE ADC.
	_instance -> SR = 0; 				// CLEAR ADC STATUS REGISTER.
#if defined(STM32F1)
	_instance -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER.
#endif /* STM32F1 */
	_instance -> CR2 |= ADC_CR2_SWSTART; // START ADC BY SOFTWARE START.
}

void adc::stop_convert(void){
	_instance -> CR2 &=~ ADC_CR2_ADON; // DISABLE ADC
#if defined(STM32F1)
	_instance -> CR2 &=~ ADC_CR2_EXTTRIG; // DISABLE ADC START BY EXTERNAL TRIGER
#endif /* STM32F1 */
	_instance -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC
}


void start_convert_it(uint16_t *pdata){

}

void stop_convert_it(void){

}


uint16_t adc::get_value(void){
	return _instance -> DR;
}

#if PERIPHERAL_DMAC_AVAILABLE
err_t adc::start_convert_dmac(uint16_t *pdata, uint16_t num_channel){
	err_t ret;

	_instance -> CR2 |= ADC_CR2_DMA; // ENABLE ADC DMA

	dmac_session_config_t dmac_s_conf = {
		.psource = (uint32_t)&_instance -> DR,
		.pdest = (uint32_t)pdata,
		.xfersize = num_channel,
		.interrupt_enable = true,
	};
	ret = _dma->config_start_session(dmac_s_conf);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	_instance -> CR2 |= ADC_CR2_ADON;
	_instance -> SR = 0;
#if defined(STM32F1)
	_instance -> CR2 |= ADC_CR2_EXTTRIG;
#endif /* STM32F1 */
	_instance -> CR2 |= ADC_CR2_SWSTART;

	return ret;
}

err_t adc::stop_convert_dmac(void){
	err_t ret;

	if(_instance -> CR2 & ADC_CR2_DMA){
		ERROR_CAPTURE(ret);
		return ret;
	}
	_instance -> CR2 &=~ ADC_CR2_DMA;

	_dma->stop_session();

	_instance -> CR2 &=~ ADC_CR2_ADON;
#if defined(STM32F1)
	_instance -> CR2 &=~ ADC_CR2_EXTTRIG; // DISABLE ADC START BY EXTERNAL TRIGER
#endif /* STM32F1 */
	_instance -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC

	return ret;
}

void adc::dmac_event_handler(dmac_event_t event, void *param){

}
#endif /* PERIPHERAL_DMAC_AVAILABLE */


static void ADCCommon_IRQHandler(adc *adc){

}


#if defined(ADC1) && defined(ADC2)
static adc adc_1(ADC1);
adc_t adc1 = &adc_1;
static adc adc_2(ADC2);
adc_t adc2 = &adc_2;

void ADC1_2_IRQHandler(void){
	ADCCommon_IRQHandler(adc1);
	ADCCommon_IRQHandler(adc2);
}
#endif /* ADC1 && ADC2 */
#if defined(ADC3)
static adc adc_3(ADC3);
adc_t adc3;
void ADC3_IRQHandler(void){
	ADCCommon_IRQHandler(adc3);
}
#endif /* ADC3 */

#endif /* PERIPHERAL_ADC_AVAILABLE */


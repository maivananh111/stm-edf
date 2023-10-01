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

#include "math.h"



static void ADCCommon_IRQHandler(adc *adc);

adc::adc(ADC_TypeDef *adc){
	_adc = adc;
}


err_t adc::adc_init(adc_config_t *conf){
	err_t ret;

	_conf = conf;
	uint32_t tmp_ADC_SMPR1 = 0;
	uint32_t tmp_ADC_SMPR2 = 0;

	/* ADC CLOCK ENABLE */
	if(_adc == ADC1) RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
#if defined(ADC2)
	if(_adc == ADC2) RCC -> APB2ENR |= RCC_APB2ENR_ADC2EN;
#endif /* defined(ADC2) */
#if defined(ADC3)
	if(_adc == ADC3) RCC -> APB2ENR |= RCC_APB2ENR_ADC3EN;
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
	_adc -> CR1 &=~ ADC_CR1_SCAN;
	if(sizeof(_conf -> pin_list) > 1)
		_adc -> CR1 |= ADC_CR1_SCAN;   // ADC SCAN MODE.

#if defined(STM32F4)
	_adc -> CR1 &=~ ADC_CR1_RES;
	_adc -> CR1 |= _conf -> resolution; // ADC RESOLUTION.
#endif /* STM32F4 */

	_adc -> CR2 &=~ ADC_CR2_ALIGN; // ADC DATA ALIGN RIGHT.
	if(_conf -> dataalign == ADC_DATAALIGN_LEFT)
		_adc -> CR2 |= ADC_CR2_ALIGN;

#if defined(STM32F1)
	_adc -> CR2 |= ADC_CR2_EXTSEL; // ADC START BY SWSTART
#elif defined(STM32F4)
	_adc -> CR2 &=~ ADC_CR2_EXTSEL; // ADC SELECT SOFTWARE START.
	_adc -> CR2 &=~ ADC_CR2_EXTEN;  // ADC DISABLE EXTERNAL TRIGER START.
#endif /* STM32F4 */

	_adc -> CR2 &=~ ADC_CR2_CONT;
	_adc -> CR2 |= _conf -> continuos; // ADC CONTINUOUS MODE ENABLE.
	_adc -> CR1 &=~ ADC_CR1_DISCEN;    // ADC DISCONTINUOUS MODE DISABLE.

	for(uint8_t i=0; i<_conf -> num_channel; i++){
		if(_conf -> rank_list[i] <= 9) tmp_ADC_SMPR2 |= (_conf -> sampletime_list[i] << (3*_conf -> rank_list[i]));      // SMPR1 REGISTER
		else 				           tmp_ADC_SMPR1 |= (_conf -> sampletime_list[i] << (3*(_conf -> rank_list[i] - 10))); // SMPR1 REGISTER
	}
	_adc -> SMPR1 = tmp_ADC_SMPR1; // ADC CHANNEL 10-17 SAMPLING TIME
	_adc -> SMPR2 = tmp_ADC_SMPR2; // ADC CHANNEL 0 - 9 SAMPLING TIME

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
	_adc -> SQR1 = ADC_SQR1; // ADC SEQUENCE NUMBER.
	_adc -> SQR2 = ADC_SQR2; // ADC SEQUENCE NUMBER.
	_adc -> SQR3 = ADC_SQR3; // ADC SEQUENCE NUMBER.

	_adc -> SQR1 |= ((_conf -> num_channel - (uint8_t)1) << ADC_SQR1_L_Pos); // ADC NUMBER OF CONVERSION SEQUENCE

	_adc -> CR2 |= ADC_CR2_ADON; // TURN ON ADC AND TO START CONVERSION

	return ret;
}

void adc::adc_start_convert(void){
	_adc -> CR2 |= ADC_CR2_ADON; // ENABLE ADC.
	_adc -> SR = 0; 				// CLEAR ADC STATUS REGISTER.
#if defined(STM32F1)
	_adc -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER.
#endif /* STM32F1 */
	_adc -> CR2 |= ADC_CR2_SWSTART; // START ADC BY SOFTWARE START.
}

void adc::adc_stop_convert(void){
	_adc -> CR2 &=~ ADC_CR2_ADON; // DISABLE ADC
#if defined(STM32F1)
	_adc -> CR2 &=~ ADC_CR2_EXTTRIG; // DISABLE ADC START BY EXTERNAL TRIGER
#endif /* STM32F1 */
	_adc -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC
}

uint16_t adc::adc_get_value(void){
	return _adc -> DR;
}

#if defined(DRIVER_DMA_SUPPORTED)
void adc::adc_add_dma(dma_drv_t _dma){
	this->_dma = _dma;
}

err_t adc::adc_start_convert_dma(uint16_t *pdata, uint16_t num_channel){
	err_t ret;

	_adc -> CR2 |= ADC_CR2_DMA; // ENABLE ADC DMA

	ret = _dma->drv_start_transfer((uint32_t)&_adc -> DR, (uint32_t)pdata,  num_channel); // SETUP DMA
	if(status_have_error(&ret)){
		STATUS_CAPTURE_LINE(ret);
		return ret;
	}

	_adc -> SR = 0; // CLEAR ADC STATUS REGISTER
#if defined(STM32F1)
	_adc -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER.
#endif /* STM32F1 */
	_adc -> CR2 |= ADC_CR2_SWSTART; // START ADC

	return ret;
}

err_t adc::adc_stop_convert_dma(void){
	err_t ret;

	if(_adc -> CR2 & ADC_CR2_DMA){
		STATUS_CAPTURE_LINE(ret);
		return ret;
	}

	_adc -> CR2 &=~ ADC_CR2_DMA; // DISABLE ADC DMA

	ret = _dma->drv_stop_transfer(); // STOP DMA
	if(status_have_error(&ret)){
		STATUS_CAPTURE_LINE(ret);
		return ret;
	}

#if defined(STM32F1)
	_adc -> CR2 &=~ ADC_CR2_EXTTRIG; // DISABLE ADC START BY EXTERNAL TRIGER
#endif /* STM32F1 */
	_adc -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC

	return ret;
}
#endif /* defined(DRIVER_DMA_SUPPORTED) */


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


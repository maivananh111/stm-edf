/*
 * adc.h
 *
 *  Created on: Jun 30, 2022
 *      Author: A315-56
 */

#ifndef PERIPHERALS_ADC_H_
#define PERIPHERALS_ADC_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_ADC_ENABLE && (defined(ADC1) || defined(ADC2) || defined(ADC3))
#define PERIPHERAL_ADC_AVAILABLE 1

#include "stdio.h"
#include "common/error_check.h"
#if CONFIG_PERIPH_DMA_ENABLE
#include "drivers/dma.h"
#endif /* ENABLE_DMA */

#ifdef __cplusplus
extern "C"{
#endif


#if defined(STM32F1)

#elif defined(STM32F4)
typedef enum{
	ADC_RESOLUTION_12BITS = 0,
	ADC_RESOLUTION_10BITS = ADC_CR1_RES_0,
	ADC_RESOLUTION_8BITS  = ADC_CR1_RES_1,
	ADC_RESOLUTION_6BITS  = ADC_CR1_RES,
} adc_resolution_t;
#endif /* STM32F4 */

typedef enum{
	ADC_PRESCALER_2 = 0,
	ADC_PRESCALER_4,
	ADC_PRESCALER_6,
	ADC_PRESCALER_8,
} adc_prescaler_t;

typedef enum{
	ADC_DATAALIGN_RIGHT = 0,
	ADC_DATAALIGN_LEFT,
} adc_dataalign_t;

typedef enum{
	ADC_CONTINUOS_DISABLE = 0,
	ADC_CONTINUOS_ENABLE = ADC_CR2_CONT,
} adc_continuosmode_t;

typedef enum{
#if defined(STM32F1)
	ADC_1_5_CYCLES   = 0UL,
	ADC_7_5_CYCLES,
	ADC_13_5_CYCLES,
	ADC_28_5_CYCLES,
	ADC_41_5_CYCLES,
	ADC_55_5_CYCLES,
	ADC_71_5_CYCLES,
	ADC_239_5_CYCLES,
#elif defined(STM32F4)
	ADC_3_CYCLES   = 0UL,
	ADC_15_CYCLES,
	ADC_28_CYCLES,
	ADC_56_CYCLES,
	ADC_84_CYCLES,
	ADC_112_CYCLES,
	ADC_144_CYCLES,
	ADC_480_CYCLES,
#endif /* defined(STM32F4) */
} adc_sampletime_t;

typedef struct{
	adc_prescaler_t prescaler = ADC_PRESCALER_4;
#if defined(STM32F4)
	adc_resolution_t resolution = ADC_RESOLUTION_12BITS;
#endif /* STM32F4 */
	adc_dataalign_t dataalign = ADC_DATAALIGN_RIGHT;
	adc_continuosmode_t continuos = ADC_CONTINUOS_DISABLE;
	GPIO_TypeDef **port_list;
	uint16_t *pin_list;
	uint8_t *rank_list;
	adc_sampletime_t *sampletime_list;
	uint8_t num_channel;
	bool adc_temp_vref = false;
} adc_config_t;

class adc{
	public:
		adc(ADC_TypeDef *adc);
		err_t adc_init(adc_config_t *conf);
#if PERIPHERAL_DMA_AVAILABLE
		void add_dma(dma_t dma = NULL);
#endif /* PERIPHERAL_DMA_AVAILABLE */

		void adc_start_convert(void);
		void adc_stop_convert(void);

		void adc_start_convert_it(uint16_t *pdata);
		void adc_stop_convert_it(void);

#if PERIPHERAL_DMA_AVAILABLE
		err_t adc_start_convert_dma(uint16_t *pdata, uint16_t num_channel);
		err_t adc_stop_convert_dma(void);
#endif /* PERIPHERAL_DMA_AVAILABLE */

		uint16_t adc_get_value(void);

	private:
		ADC_TypeDef *_adc;
		adc_config_t *_conf = NULL;
#if PERIPHERAL_DMA_AVAILABLE
		dma_t _dma = NULL;
#endif /* PERIPHERAL_DMA_AVAILABLE */
};

typedef adc* adc_t;

#if defined(ADC1) && defined(ADC2)
extern adc_t adc1;
extern adc_t adc2;
void ADC1_2_IRQHandler(void);
#endif /* ADC1 && ADC2 */
#if defined(ADC3)
extern adc_t adc3;
void ADC3_IRQHandler(void);
#endif /* ADC3 */

#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_ADC_AVAILABLES 0
#endif /* CONFIG_PERIPH_ADC_ENABLE && (defined(ADC1) || defined(ADC2) || defined(ADC3)) */

#endif /* PERIPHERALS_ADC_H_ */

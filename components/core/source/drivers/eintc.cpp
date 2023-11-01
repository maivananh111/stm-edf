/*
 * eintc.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#include "drivers/eintc.h"
#if PERIPHERAL_EINTC_AVAILABLE

#include "stdio.h"
#include "drivers/sysinfo.h"
#include "drivers/system.h"
#include "common/macro.h"
#if CONFIG_PERIPH_EINTC_LOG
#include "common/log_monitor.h"
#endif /* CONFIG_PERIPH_EINTC_LOG */

#define EINTC_LINE_INDEX 6U

#if CONFIG_PERIPH_EINTC_LOG
static const char *TAG = "EINTC";
#endif /* CONFIG_PERIPH_EINTC_LOG */


#if defined(STM32F1)
#define EINTC_CONFIG_REGISTER AFIO
#define EINTC_GPIO_ADRRESS_OFFSET 0x0800UL
#elif defined(STM32F4)
#define EINTC_CONFIG_REGISTER SYSCFG
#define EINTC_GPIO_ADRRESS_OFFSET 0x0000UL
#endif /* STM32F4 */
#define EINTC EXTI

static eintc_event_handler_f _event_handler[16] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static void *_event_parameter[16] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};


static inline void EINTCCommon_IRQHandler(uint16_t Pin);

err_t eintc_line_initialize(GPIO_TypeDef *port, uint16_t pin, eintc_edge_t edge, uint32_t priority){
	err_t ret;
	uint8_t CRPos = 0;
	IRQn_Type IRQn;

	if(priority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
#if CONFIG_PERIPH_EINTC_LOG
		LOG_MESS(LOG_ERROR, TAG, "Interrupt priority out of range");
#endif /* CONFIG_PERIPH_SPI_LOG */
		system_reset();
		return ret;
	}

	if(pin < 4U) 					CRPos = 0;
	else if(pin >= 4U && pin < 8U)  CRPos = 1;
	else if(pin >= 8U && pin < 12U) CRPos = 2;
	else 							CRPos = 3;

	if(pin < 5U) IRQn = (IRQn_Type)(pin + EINTC_LINE_INDEX);
	else if(pin >= 5U && pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;

#if defined(STM32F1)
	if(!(RCC->APB2ENR & RCC_APB2ENR_AFIOEN))   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
#elif defined(STM32F4)
	if(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN)) SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
#endif /* STM32F4 */
	__IO uint32_t tmpreg = READ_REG(EINTC_CONFIG_REGISTER->EXTICR[CRPos]);

	tmpreg &=~ (0x0F << ((pin - CRPos*4U) * 4U));
	tmpreg |= (uint32_t)(((((uint32_t)((uint32_t)(port) - EINTC_GPIO_ADRRESS_OFFSET) & 0xFF00U) >> 8U) / 4U) << ((pin - CRPos*4U) * 4U));

	WRITE_REG(EINTC_CONFIG_REGISTER->EXTICR[CRPos], tmpreg);

	if(edge & EXTI_RTSR_TR0) SET_BIT(EINTC->RTSR, (1U << pin));
	if(edge & EXTI_FTSR_TR0) SET_BIT(EINTC->FTSR, (1U << pin));

	SET_BIT(EINTC->IMR, (1U << pin));

	__NVIC_SetPriority(IRQn, priority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY);
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	return ret;
}

void eintc_line_deinitialize(GPIO_TypeDef *port, uint16_t pin){
	uint8_t CRPos = 0;

	if(pin < 4U) 					CRPos = 0;
	else if(pin >= 4U && pin < 8U)  CRPos = 1;
	else if(pin >= 8U && pin < 12U) CRPos = 2;
	else 							CRPos = 3;

	WRITE_REG(EINTC->PR, (1U << pin));
	CLEAR_BIT(EINTC_CONFIG_REGISTER->EXTICR[CRPos], (0x0F << ((pin - CRPos*4U) * 4U)));

	CLEAR_BIT(EINTC->RTSR, (1U << pin));
	CLEAR_BIT(EINTC->FTSR, (1U << pin));

	CLEAR_BIT(EINTC->IMR, (1U << pin));

	IRQn_Type IRQn;
	if(pin < 5U) IRQn = (IRQn_Type)(pin + EINTC_LINE_INDEX);
	else if(pin >= 5U && pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);
}

void eintc_line_register_event_handler(uint16_t pin, eintc_event_handler_f event_handler_function, void *param){
	_event_handler[pin] = event_handler_function;
	_event_parameter[pin] = param;
}

void eintc_line_unregister_event_handler(uint16_t pin){
	_event_handler[pin] = NULL;
	_event_parameter[pin] = NULL;
}


static inline void EINTCCommon_IRQHandler(uint16_t Pin){
	if(READ_BIT(EINTC->PR, (1U << Pin))){
		WRITE_REG(EINTC->PR, (1U << Pin));
		if(_event_handler[Pin] != NULL) _event_handler[Pin](_event_parameter[Pin]);
	}
}


void EXTI0_IRQHandler(void){
	EINTCCommon_IRQHandler(0);
}

void EXTI1_IRQHandler(void){
	EINTCCommon_IRQHandler(1);
}

void EXTI2_IRQHandler(void){
	EINTCCommon_IRQHandler(2);
}

void EXTI3_IRQHandler(void){
	EINTCCommon_IRQHandler(3);
}

void EXTI4_IRQHandler(void){
	EINTCCommon_IRQHandler(4);
}

void EXTI9_5_IRQHandler(void){
	EINTCCommon_IRQHandler(5);
	EINTCCommon_IRQHandler(6);
	EINTCCommon_IRQHandler(7);
	EINTCCommon_IRQHandler(8);
	EINTCCommon_IRQHandler(9);
}

void EXTI15_10_IRQHandler(void){
	EINTCCommon_IRQHandler(10);
	EINTCCommon_IRQHandler(11);
	EINTCCommon_IRQHandler(12);
	EINTCCommon_IRQHandler(13);
	EINTCCommon_IRQHandler(14);
	EINTCCommon_IRQHandler(15);
}



#endif /* DRIVER_EXTI_SUPPORTED */












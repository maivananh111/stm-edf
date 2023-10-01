/*
 * exti.h
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_EXTI_H_
#define PERIPHERALS_EXTI_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_EXTI_ENABLE
#define PERIPHERAL_EXTI_AVAILABLE 1

#include "iostream"
#include "functional"

#include "common/error_check.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
	EXTI_RISING_EDGE  = EXTI_RTSR_TR0,
	EXTI_FALLING_EDGE = EXTI_FTSR_TR0,
	EXTI_RISING_FALLING_EDGE  = EXTI_RTSR_TR0 | EXTI_FTSR_TR0,
} exti_edgedetect_t;

typedef void(*exti_evcb_t)(void *);

err_t exti_line_initialize(GPIO_TypeDef *port, uint16_t pin, exti_edgedetect_t edge, uint32_t priority);
void exti_line_deinitialize(GPIO_TypeDef *port, uint16_t pin);

void exti_line_register_event_handler(uint16_t pin, std::function<void(void *)> event_handler_function, void *param = NULL);
void exti_line_unregister_event_handler(uint16_t pin);


void EXTI0_IRQHandler(void);           /* EXTI Line0 interrupt */
void EXTI1_IRQHandler(void);           /* EXTI Line1 interrupt */
void EXTI2_IRQHandler(void);           /* EXTI Line2 interrupt */
void EXTI3_IRQHandler(void);           /* EXTI Line3 interrupt */
void EXTI4_IRQHandler(void);           /* EXTI Line4 interrupt */
void EXTI9_5_IRQHandler(void);         /* EXTI Line[9:5] interrupts */
void EXTI15_10_IRQHandler(void);       /* EXTI Line[15:10] interrupts */


#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_EXTI_AVAILABLE 0
#endif /* ENABLCONFIG_PERIPH_EXTI_ENABLEE_EXTI */

#endif /* PERIPHERALS_EXTI_H_ */

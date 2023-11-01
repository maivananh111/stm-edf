/*
 * eintc.h
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_EINTC_H_
#define PERIPHERALS_EINTC_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_EINTC_EN
#define PERIPHERAL_EINTC_AVAILABLE 1

#include "iostream"
#include "functional"

#include "common/error_check.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
	EINTC_RISING_EDGE  = EXTI_RTSR_TR0,
	EINTC_FALLING_EDGE = EXTI_FTSR_TR0,
	EINTC_RISING_FALLING_EDGE  = EXTI_RTSR_TR0 | EXTI_FTSR_TR0,
} eintc_edge_t;

typedef void(*eintc_evcb_t)(void *);

err_t eintc_line_initialize(GPIO_TypeDef *port, uint16_t pin, eintc_edge_t edge, uint32_t priority);
void eintc_line_deinitialize(GPIO_TypeDef *port, uint16_t pin);

void eintc_line_register_event_handler(uint16_t pin, std::function<void(void *)> event_handler_function, void *param = NULL);
void eintc_line_unregister_event_handler(uint16_t pin);


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
#endif /* ENABLCONFIG_PERIPH_EXTI_ENE_EXTI */

#endif /* PERIPHERALS_EXTI_H_ */

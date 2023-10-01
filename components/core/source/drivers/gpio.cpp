/*
 * gpio.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */
#include "drivers/gpio.h"
#include "stdlib.h"
#include "string.h"




void gpio_port_clock_enable(GPIO_TypeDef *port){
#if defined(GPIOA)
	if(port == GPIOA) {GPIOA_CLOCKENABLE(); return;}
#endif /* defined(GPIOA */
#if defined(GPIOB)
	if(port == GPIOB) {GPIOB_CLOCKENABLE(); return;}
#endif /* defined(GPIOB */
#if defined(GPIOC)
	if(port == GPIOC) {GPIOC_CLOCKENABLE(); return;}
#endif /* defined(GPIOC */
#if defined(GPIOD)
	if(port == GPIOD) {GPIOD_CLOCKENABLE(); return;}
#endif /* defined(GPIOD */
#if defined(GPIOE)
	if(port == GPIOE) {GPIOE_CLOCKENABLE(); return;}
#endif /* defined(GPIOE */
#if defined(GPIOF)
	if(port == GPIOF) {GPIOF_CLOCKENABLE(); return;}
#endif /* defined(GPIOF */
#if defined(GPIOG)
	if(port == GPIOG) {GPIOG_CLOCKENABLE(); return;}
#endif /* defined(GPIOG */
#if defined(GPIOH)
	if(port == GPIOH) {GPIOH_CLOCKENABLE(); return;}
#endif /* defined(GPIOH */
#if defined(GPIOI)
	if(port == GPIOI) {GPIOI_CLOCKENABLE(); return;}
#endif /* defined(GPIO */
#if defined(GPIOJ)
	if(port == GPIOJ) {GPIOJ_CLOCKENABLE(); return;}
#endif /* defined(GPIO */
#if defined(GPIOK)
	if(port == GPIOK) {GPIOK_CLOCKENABLE(); return;}
#endif /* defined(GPIO */
}

void gpio_allport_clock_enable(void){
#if defined(GPIOA)
	GPIOA_CLOCKENABLE();
#endif /* defined(GPIOA */
#if defined(GPIOB)
	GPIOB_CLOCKENABLE();
#endif /* defined(GPIOB */
#if defined(GPIOC)
	GPIOC_CLOCKENABLE();
#endif /* defined(GPIOC */
#if defined(GPIOD)
	GPIOD_CLOCKENABLE();
#endif /* defined(GPIOD */
#if defined(GPIOE)
	GPIOE_CLOCKENABLE();
#endif /* defined(GPIOE */
#if defined(GPIOF)
	GPIOF_CLOCKENABLE();
#endif /* defined(GPIOF */
#if defined(GPIOG)
	GPIOG_CLOCKENABLE();
#endif /* defined(GPIOG */
#if defined(GPIOH)
	GPIOH_CLOCKENABLE();
#endif /* defined(GPIOH */
#if defined(GPIOI)
	GPIOI_CLOCKENABLE();
#endif /* defined(GPIOI */
#if defined(GPIOJ)
	GPIOJ_CLOCKENABLE();
#endif /* defined(GPIOJ */
#if defined(GPIOK)
	GPIOK_CLOCKENABLE();
#endif /* defined(GPIOJ */
}


bool gpio_str_decode(char *str, gpio_pin_t *ppin){
	if(str == NULL || ppin == NULL) return false;

	if(str[0] >= 'A' && str[0] <= 'K' && str[1] >= '0' && str[1] <= '9'){
#if defined(STM32F1)
		ppin->port = (GPIO_TypeDef *)(((str[0] - 0x41UL) * 0x00000400UL) + APB2PERIPH_BASE + 0x00000800UL);
#elif defined(STM32F4)
		ppin->port = (GPIO_TypeDef *)(((str[0] - 0x41UL) * 0x00000400UL) + AHB1PERIPH_BASE);
#endif
		ppin->pinnum = atoi((char *)(str + 1));
		return true;
	}
	ppin->port = NULL;
	ppin->pinnum = -1;
	return false;
}


void gpio_init(gpio_config_t *conf){
#if defined(STM32F1)
	__IO uint32_t *GPIO_CTRL_REG;
	__IO int8_t pin;

	if(conf->pinnum < 8){
		GPIO_CTRL_REG = (__IO uint32_t *)conf->port->CRL;
		pin = conf->pinnum;
	}
	else{
		GPIO_CTRL_REG = (__IO uint32_t *)conf->port->CRH;
		pin = conf->pinnum-8U;
	}

	if(conf->direction == GPIO_INPUT){
		CLEAR_BIT(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(pin*4)));  /** MODE[0:1] */
		(conf->pullresistor == GPIO_PULL_NONE)?    /** CNF[0:1] */
			MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+pin*4)), (1U<<(2+pin*4)))
		  : MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+pin*4)), (2U<<(2+pin*4)));
	}
	else if(conf->direction == GPIO_ANALOG){
		CLEAR_BIT(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(pin*4)));     /** MODE[0:1] */
		CLEAR_BIT(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4)))); /** CNF[0:1] */
	}
	else{
		MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(pin*4)), ((conf->outputspeed+1U)<<(pin*4)));
		if(conf->direction == GPIO_OUTPUT){
			(conf->type == GPIO_OUTPUT_PUSHPULL)?
					CLEAR_BIT(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))))
				  : MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (1U<<(2+(pin*4))));
		}
		else if(conf->direction == GPIO_FUNCTION){
			(conf->type == GPIO_OUTPUT_PUSHPULL)?
					MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (2U<<(2+(pin*4))))
				  : MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (3U<<(2+(pin*4))));
		}
	}

	if(conf->pullresistor == GPIO_PULL_UP) SET_BIT(conf->port->ODR, (1U<<conf->pinnum));
	else if(conf->pullresistor == GPIO_PULL_DOWN) CLEAR_BIT(conf->port->ODR, (1U<<conf->pinnum));
#elif defined(STM32F4)

	MODIFY_REG(conf->port->MODER,   (3U<<(conf->pinnum*2U)), (conf->direction<<(conf->pinnum*2U)));
	MODIFY_REG(conf->port->OTYPER,  (1U<<(conf->pinnum)),    (conf->type<<conf->pinnum));
	MODIFY_REG(conf->port->OSPEEDR, (3U<<(conf->pinnum*2U)), (conf->outputspeed<<(conf->pinnum*2U)));
	MODIFY_REG(conf->port->PUPDR,   (3U<<(conf->pinnum*2)),  (conf->pullresistor<<(conf->pinnum*2)));

	if(conf->direction == GPIO_FUNCTION){
		(conf->pinnum < 8)?
			MODIFY_REG(conf->port->AFR[0], (0x0FU<<(conf->pinnum*4)), (conf->function <<(conf->pinnum*4)))
		  : MODIFY_REG(conf->port->AFR[1], (0x0FU<<((conf->pinnum-8)*4)), (conf->function <<((conf->pinnum-8)*4)));
	}
#endif /* STM32F4 */
}


void gpio_deinit(GPIO_TypeDef *port, int8_t pinnum){
#if defined(STM32F1)
	if(pinnum < 8){
		MODIFY_REG(port->CRL, (0x0FU<<(pinnum*4U)), (0x04U<<(pinnum*4U)));
	}
	else{
		MODIFY_REG(port->CRL, (0x0FU<<((pinnum-8)*4U)), (0x04U<<((pinnum-8)*4U)));
	}
	CLEAR_BIT(port->ODR, (1U<<pinnum));
#elif defined(STM32F4)
	if((port == GPIOA && pinnum >= 13) || (port == GPIOB && (pinnum == 3 || pinnum == 4)))
		MODIFY_REG(port->MODER, (2U<<(pinnum*2)), (2U<<(pinnum*2)));

	CLEAR_BIT(port->OTYPER, (1U<<pinnum));

	if((port == GPIOA && pinnum == 13U) || (port == GPIOB && pinnum == 3U))
		SET_BIT(port->OSPEEDR, (3U<<(pinnum*2)));

	if(port == GPIOA){
		if(pinnum == 13 || pinnum == 15) MODIFY_REG(port->PUPDR, (3U<<(pinnum*2)), (1U<<(pinnum*2)));
		else if(pinnum == 14)            MODIFY_REG(port->PUPDR, (3U<<(pinnum*2)), (2U<<(pinnum*2)));
	}
	if(port == GPIOB && pinnum == 4) MODIFY_REG(port->PUPDR, (3U<<(pinnum*2)), (1U<<(pinnum*2)));

	if(pinnum < 8) CLEAR_BIT(port->AFR[0], (0x0FU<<(pinnum*4)));
	else           CLEAR_BIT(port->AFR[1], (0x0FU<<((pinnum-8)*4)));
#endif /* STM32F4 */

	SET_BIT(port->BSRR, ((1U<<pinnum)<<16));
}


void gpio_set_direction(GPIO_TypeDef *port, int8_t pinnum, gpio_direction_t dir){
	gpio_config_t conf = GPIO_CONFIG_DEFAULT();
	conf.port = port;
	conf.pinnum = pinnum;
	conf.direction = dir;
	gpio_init(&conf);
}



#if defined(STM32F4)
void gpio_set_function(GPIO_TypeDef *port, int8_t pinnum, gpio_function_t function){
	(pinnum < 8)?
		MODIFY_REG(port->AFR[0], (0x0FU<<(pinnum*4)),     (function<<(pinnum*4)))
	  : MODIFY_REG(port->AFR[1], (0x0FU<<((pinnum-8)*4)), (function<<((pinnum-8)*4)));
}
#endif /* STM32F4 */

void gpio_set_type(GPIO_TypeDef *port, int8_t pinnum, gpio_type_t type){
#if defined(STM32F1)
	__IO uint32_t *GPIO_CTRL_REG;
	__IO int8_t pin;

	if(pinnum < 8){
		GPIO_CTRL_REG = (__IO uint32_t *)port->CRL;
		pin = pinnum;
	}
	else{
		GPIO_CTRL_REG = (__IO uint32_t *)port->CRH;
		pin = pinnum-8U;
	}

	if(type == GPIO_OUTPUT_PUSHPULL)
		CLEAR_BIT(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))));
	else if(type == GPIO_OUTPUT_OPENDRAIN)
		MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (1U<<(2+(pin*4))));
	else if(type == GPIO_OUTPUT_PUSHPULL)
		MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (2U<<(2+(pin*4))));
	else
		MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(2+(pin*4))), (3U<<(2+(pin*4))));

#elif defined(STM32F4)
	if(type == GPIO_OUTPUT_OPENDRAIN     || type == GPIO_FUNCTION_OPENDRAIN) SET_BIT(port->OTYPER,   (1U<<pinnum));
	else if(type == GPIO_OUTPUT_PUSHPULL || type == GPIO_FUNCTION_PUSHPULL)  CLEAR_BIT(port->OTYPER, (1U<<pinnum));
#endif /* STM32F4 */
}


void gpio_set_outputspeed(GPIO_TypeDef *port, int8_t pinnum, gpio_outputspeed_t outputspeed){
#if defined(STM32F1)
	__IO uint32_t *GPIO_CTRL_REG;
	__IO int8_t pin;

	if(pinnum < 8){
		GPIO_CTRL_REG = (__IO uint32_t *)port->CRL;
		pin = pinnum;
	}
	else{
		GPIO_CTRL_REG = (__IO uint32_t *)port->CRH;
		pin = pinnum-8U;
	}
	MODIFY_REG(*(__IO uint32_t *)GPIO_CTRL_REG, (3U<<(pin*4)), ((outputspeed+1U)<<(pin*4)));
#elif defined(STM32F4)
	MODIFY_REG(port->OSPEEDR, (3U<<(pinnum*2U)), (outputspeed<<(pinnum*2U)));
#endif
}


void gpio_set_pullup(GPIO_TypeDef *port, int8_t pinnum){
#if defined(STM32F1)
	SET_BIT(port->ODR, (1<<pinnum));
#elif defined(STM32F4)
	MODIFY_REG(port ->PUPDR, (3U<<(pinnum*2)), (1U<<(pinnum*2)));
#endif /* STM32F4 */
}

void gpio_set_pulldown(GPIO_TypeDef *port, int8_t pinnum){
#if defined(STM32F1)
	CLEAR_BIT(port->ODR, (1<<pinnum));
#elif defined(STM32F4)
	MODIFY_REG(port ->PUPDR, (3U<<(pinnum*2)), (2U<<(pinnum*2)));
#endif /* STM32F4 */
}

#if defined(STM32F1)
void gpio_remap(gpio_remap_t remap){
	SET_BIT(AFIO->MAPR, remap);
}
#endif /* STM32F1 */





void gpio_high(GPIO_TypeDef *port, int8_t pinnum){
	SET_BIT(port->BSRR, (1<<pinnum));
}

void gpio_low(GPIO_TypeDef *port, int8_t pinnum){
	SET_BIT(port->BSRR, (1<<(pinnum + 16)));
}

void gpio_toggle(GPIO_TypeDef *port, int8_t pinnum){
	(READ_BIT(port->ODR, (1<<pinnum)))?
			SET_BIT(port->BSRR, (1<<(pinnum + 16U)))
		  : SET_BIT(port->BSRR, (1<<pinnum));
}

void gpio_set_level(GPIO_TypeDef *port, int8_t pinnum, int level){
	(level)?
			SET_BIT(port->BSRR, (1<<pinnum))
		  : SET_BIT(port->BSRR, (1<<(pinnum + 16U)));
}

int gpio_get_level(GPIO_TypeDef *port, int8_t pinnum){
	return (READ_REG(port->IDR) >> pinnum) & 0x01UL;
}








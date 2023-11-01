/*
 * gpio.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_GPIO_H_
#define PERIPHERALS_GPIO_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE

#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
	GPIO_INPUT     = 0x00U,
	GPIO_OUTPUT    = 0x01U,
	GPIO_FUNCTION  = 0x02U,
	GPIO_ANALOG    = 0x03U,
} gpio_direction_t;

typedef enum{
	GPIO_OUTPUT_PUSHPULL    = 0x00U,
	GPIO_OUTPUT_OPENDRAIN   = 0x01U,
#if defined(STM32F1)
	GPIO_FUNCTION_PUSHPULL  = 0x02U,
	GPIO_FUNCTION_OPENDRAIN = 0x03U,
#elif defined(STM32F4)
	GPIO_FUNCTION_PUSHPULL  = 0x00U,
	GPIO_FUNCTION_OPENDRAIN = 0x01U,
#endif
} gpio_type_t;

typedef enum{
	GPIO_OUTPUT_LOW_SPEED = 0,
	GPIO_OUTPUT_MEDIUM_SPEED,
	GPIO_OUTPUT_HIGH_SPEED,
#if defined(STM32F4)
	GPIO_OUTPUT_VERYHIGH_SPEED,
#endif /* STM32F4 */
} gpio_outputspeed_t;

typedef enum{
	GPIO_PULL_NONE = 0,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
} gpio_pullresistor_t;

#if defined(STM32F4)
typedef enum{
	AF0_SYSTEM = 0U,
	AF1_TIM1_2,
	AF2_TIM3_5,
	AF3_TIM8_11,
	AF4_I2C1_3,
#if defined(SPI4) & defined(SPI5) & defined(SPI6)
	AF5_SPI1_6,
#else
	AF5_SPI1_2,
#endif /* defined(SPI4) & defined(SPI5) & defined(SPI6) */
#if defined(SAI1) & defined(SPI2) & defined(SPI3)
	AF6_SPI2_3_SAI1,
#else
	AF6_SPI3,
#endif /* defined(SAI1) & defined(SPI2) & defined(SPI3) */
	AF7_UART1_3,
#if defined(UART7) && defined(UART8)
	AF8_UART4_8,
#else
	AF8_UART4_6,
#endif /* defined(UART7) && defined(UART8) */

#if defined(LTDC)
	AF9_CAN1_2_LTDC_TIM12_14, // ONLY AFRL.
#else
	AF9_CAN1_2_TIM12_14,
#endif /* defined(LTDC) */
	AF10_USB,
	AF11_ETH,
	AF12_FMC_FSMC_SDIO_USB,
	AF13_DCMI,
#if defined(LTDC)
	AF14_LTDC,
#else
	AF14,
#endif /* defined(LTDC) */
	AF15_EVENTOUT,
} gpio_function_t;
#endif /* STM32F4 */

#if defined(STM32F1)
typedef enum{
	SPI1_Remap 			= AFIO_MAPR_SPI1_REMAP,
	I2C1_Remap 			= AFIO_MAPR_I2C1_REMAP,
	USART1_Remap 		= AFIO_MAPR_USART1_REMAP,
	USART2_Remap 		= AFIO_MAPR_USART2_REMAP,
	USART3_Remap_PC 	= AFIO_MAPR_USART3_REMAP_PARTIALREMAP,
	USART3_Remap_PD 	= AFIO_MAPR_USART3_REMAP_FULLREMAP,
	TIM1_Partial_Remap  = AFIO_MAPR_TIM1_REMAP_PARTIALREMAP,
	TIM1_Full_Remap 	= AFIO_MAPR_TIM1_REMAP_FULLREMAP,
	TIM2_Partial_Remap1 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1,
	TIM2_Partial_Remap2 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2,
	TIM2_Full_Remap     = AFIO_MAPR_TIM2_REMAP_FULLREMAP,
	TIM3_Partial_Remap  = AFIO_MAPR_TIM3_REMAP_PARTIALREMAP,
	TIM3_Full_Remap     = AFIO_MAPR_TIM3_REMAP_FULLREMAP,
	TIM4_Remap     		= AFIO_MAPR_TIM4_REMAP,
	CAN_Remap_PB 		= AFIO_MAPR_CAN_REMAP_REMAP2,
	CAN_Remap_PD 		= AFIO_MAPR_CAN_REMAP_REMAP3,
} gpio_remap_t;
#endif /* STM32F1 */




#if CONFIG_PERIPH_GPIO_DEFAULT_OUTPUT_SPEED_LOW
#define GPIO_OUTPUTSPEED_DEFAULT GPIO_OUTPUT_LOW_SPEED
#elif CONFIG_PERIPH_GPIO_DEFAULT_OUTPUT_SPEED_MEDIUM
#define GPIO_OUTPUTSPEED_DEFAULT GPIO_OUTPUT_MEDIUM_SPEED
#elif CONFIG_PERIPH_GPIO_DEFAULT_OUTPUT_SPEED_HIGH
#define GPIO_OUTPUTSPEED_DEFAULT GPIO_OUTPUT_HIGH_SPEED
#else
#define GPIO_OUTPUTSPEED_DEFAULT GPIO_OUTPUT_VERYHIGH_SPEED
#endif

#if CONFIG_PERIPH_GPIO_DEFAULT_OUTPUT_PUSHPULL
#define GPIO_OUTPUTTYPE_DEFAULT GPIO_OUTPUT_PUSHPULL
#elif CONFIG_PERIPH_GPIO_DEFAULT_OUTPUT_OPENDRAIN
#define GPIO_OUTPUTTYPE_DEFAULT GPIO_OUTPUT_OPENDRAIN
#endif



typedef struct{
	GPIO_TypeDef               *port = GPIOA;
	int8_t                    pinnum = -1;
	gpio_direction_t       direction = GPIO_INPUT;
#if defined(STM32F4)
	gpio_function_t         function = AF0_SYSTEM;
#endif
	gpio_type_t                 type = GPIO_OUTPUTTYPE_DEFAULT;
	gpio_outputspeed_t   outputspeed = GPIO_OUTPUTSPEED_DEFAULT;
	gpio_pullresistor_t pullresistor = GPIO_PULL_NONE;
} gpio_config_t;

typedef struct{
	GPIO_TypeDef *port = GPIOA;
	int8_t      pinnum = -1;
} gpio_pin_t;

#define GPIO_CONFIG_DEFAULT()   			 \
{											 \
	.direction    = GPIO_INPUT,				 \
	.type         = GPIO_OUTPUTTYPE_DEFAULT, \
	.outputspeed  = GPIO_OUTPUTSPEED_DEFAULT,\
	.pullresistor = GPIO_PULL_NONE           \
}



bool gpio_str_decode(char *str, gpio_pin_t *ppin);


void gpio_allport_clock_enable(void);
void gpio_port_clock_enable(GPIO_TypeDef *port);


void gpio_init(gpio_config_t *conf);
void gpio_deinit(GPIO_TypeDef *port, int8_t pinnum);


void gpio_set_direction(GPIO_TypeDef *port, int8_t pinnum, gpio_direction_t dir);
#if defined(STM32F4)
void gpio_set_function(GPIO_TypeDef *port, int8_t pinnum, gpio_function_t function);
#endif
void gpio_set_type(GPIO_TypeDef *port, int8_t pinnum, gpio_type_t type);
void gpio_set_outputspeed(GPIO_TypeDef *port, int8_t pinnum, gpio_outputspeed_t outputspeed);
void gpio_set_pullup(GPIO_TypeDef *port, int8_t pinnum);
void gpio_set_pulldown(GPIO_TypeDef *port, int8_t pinnum);

#if defined(STM32F1)
void gpio_remap(gpio_remap_t remap);
#endif /* STM32F1 */

void gpio_high(GPIO_TypeDef *port, int8_t pinnum);
void gpio_low(GPIO_TypeDef *port, int8_t pinnum);
void gpio_toggle(GPIO_TypeDef *port, int8_t pinnum);


void gpio_set_level(GPIO_TypeDef *port, int8_t pinnum, int level);
int gpio_get_level(GPIO_TypeDef *port, int8_t pinnum);


#if defined(GPIOA)
#if defined(STM32F1)
#define GPIOA_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN
#elif defined(STM32F4)
#define GPIOA_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#endif /* STM32F4 */
#endif /* defined(GPIOA */
#if defined(GPIOB)
#if defined(STM32F1)
#define GPIOB_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN
#elif defined(STM32F4)
#define GPIOB_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#endif /* STM32F4 */
#endif /* defined(GPIOB */
#if defined(GPIOC)
#if defined(STM32F1)
#define GPIOC_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN
#elif defined(STM32F4)
#define GPIOC_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#endif /* STM32F4 */
#endif /* defined(GPIOC */
#if defined(GPIOD)
#if defined(STM32F1)
#define GPIOD_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPDEN
#elif defined(STM32F4)
#define GPIOD_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN
#endif /* STM32F4 */
#endif /* defined(GPIOD */
#if defined(GPIOE)
#if defined(STM32F1)
#define GPIOE_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPEEN
#elif defined(STM32F4)
#define GPIOE_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#endif /* STM32F4 */
#endif /* defined(GPIOE */
#if defined(GPIOF)
#if defined(STM32F1)
#define GPIOF_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPFEN
#elif defined(STM32F4)
#define GPIOF_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOFEN
#endif /* STM32F4 */
#endif /* defined(GPIOF */
#if defined(GPIOG)
#if defined(STM32F1)
#define GPIOG_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPGEN
#elif defined(STM32F4)
#define GPIOG_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOGEN
#endif /* STM32F4 */
#endif /* defined(GPIOG */
#if defined(GPIOH)
#if defined(STM32F1)
#define GPIOH_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPHEN
#elif defined(STM32F4)
#define GPIOH_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOHEN
#endif /* STM32F4 */
#endif /* defined(GPIOH */
#if defined(GPIOI)
#if defined(STM32F1)
#define GPIOI_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPIEN
#elif defined(STM32F4)
#define GPIOI_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOIEN
#endif /* STM32F4 */
#endif /* defined(GPIOI */
#if defined(GPIOJ)
#if defined(STM32F1)
#define GPIOJ_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPJEN
#elif defined(STM32F4)
#define GPIOJ_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOJEN
#endif /* STM32F4 */
#endif /* defined(GPIOJ */
#if defined(GPIOK)
#if defined(STM32F1)
#define GPIOK_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPKEN
#elif defined(STM32F4)
#define GPIOK_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOKEN
#endif /* STM32F4 */
#endif /* defined(GPIOK */

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_GPIO_H_ */

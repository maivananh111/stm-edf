/*
 * tim.h
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_TIM_H_
#define PERIPHERALS_TIM_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if (CONFIG_PERIPH_TIM_EN && (defined(TIM1) || defined(TIM2) || defined(TIM3) || defined(TIM4) || \
					defined(TIM5) || defined(TIM6) || defined(TIM7) || defined(TIM8) || \
					defined(TIM9) || defined(TIM10) || defined(TIM11) || defined(TIM12) || \
					defined(TIM13) || defined(TIM14)))
#define PERIPHERAL_TIM_AVAILABLE 1

#include "iostream"
#include "functional"

#include "stdio.h"
#include "common/error_check.h"
#include "common/expression_check.h"
#if CONFIG_PERIPH_DMAC_EN
#include "drivers/dmac.h"
#endif
#include "drivers/gpio.h"

#ifdef __cplusplus
extern "C"{
#endif


// BASE TIMER ENUM - STRUCT.
typedef enum{
	TIM_CHANNEL1 = 0U,
	TIM_CHANNEL2,
	TIM_CHANNEL3,
	TIM_CHANNEL4,
	TIM_NOCHANNEL,
} tim_channel_t;

typedef enum{
	TIM_COUNTER_UP = 0U,
	TIM_COUNTER_DOWN,
} tim_direction_t;

typedef enum{
	TIM_EDGE_ALIGN = 0U,
	TIM_CENTER_MODE1,
	TIM_CENTER_MODE2,
	TIM_CENTER_MODE3,
} tim_align_t;

typedef enum{
	TIM_ARP_DISABLE = 0U,
	TIM_ARP_EN,
} tim_arpe_t;

typedef enum{
	TIM_RISING_EDGE = 0U,
	TIM_FALLING_EDGE = 0x01U,
	TIM_BOTH_EDGE = 0x05U,
} tim_polarity_t;


typedef enum{
	TIM_EVENT_UPDATE,
	TIM_EVENT_BREAK,
	TIM_EVENT_TRIGER,
	TIM_EVENT_CAPTURECOMPARE1,
	TIM_EVENT_CAPTURECOMPARE2,
	TIM_EVENT_CAPTURECOMPARE3,
	TIM_EVENT_CAPTURECOMPARE4,
	TIM_EVENT_NOEVENT,
} tim_event_t;

typedef struct{
	uint32_t 		prescaler = 0U;
	uint32_t 	    reload = 0U;
	tim_direction_t direction =TIM_COUNTER_UP;
	tim_align_t     align = TIM_EDGE_ALIGN;
	tim_arpe_t		autoreloadpreload = TIM_ARP_DISABLE;
	uint32_t        interruptpriority = 0U;
} tim_config_t;


// TIMER PWM MODE ENUM - STRUCT.
typedef enum{
	TIM_PWM_NOINVERT = 6U,
	TIM_PWM_INVERT = 7U,
} tim_pwm_invert_t;

typedef enum{
	TIM_PRELOAD_DISABLE = 0U,
	TIM_PRELOAD_EN,
} tim_preload_t;

typedef enum{
	TIM_FASTMODE_DISABLE = 0U,
	TIM_FASTMODE_EN,
} tim_fast_mode_t;


typedef struct {
	char  		     *pin;
	tim_pwm_invert_t invert = TIM_PWM_NOINVERT;
	tim_preload_t 	 preload = TIM_PRELOAD_DISABLE;
	tim_fast_mode_t  fastmode = TIM_FASTMODE_DISABLE;
	// Input mode.
	tim_polarity_t   polarity = TIM_RISING_EDGE;
	uint8_t 		 ch1_filter = 0U;
	uint8_t 		 ch2_filter = 0U;
} tim_config_pwm_t;

// TIMER ENCODER MODE ENUM - STRUCT.
typedef enum {
	TIM_ENCODER_MODE1 = 1U,
	TIM_ENCODER_MODE2 = 2U,
	TIM_ENCODER_MODE3 = 3U,
} tim_encoder_mode_t;

typedef struct {
	tim_encoder_mode_t mode = TIM_ENCODER_MODE1;
	char 		   	   *encA_ch1_pin;
	char 		       *encB_ch2_pin;
	tim_polarity_t 	   encA_ch1_edge = TIM_RISING_EDGE;
	tim_polarity_t 	   encB_ch2_edge = TIM_RISING_EDGE;
	uint8_t 		   encA_ch1_filter = 0U;
	uint8_t 		   encB_ch2_filter = 0U;
	uint8_t 		   encA_ch1_prescaler = 0U;
	uint8_t 		   encB_ch2_prescaler = 0U;
} tim_config_encoder_t;

// TIMER INPUT CAPTURE MODE ENUM - STRUCT.
typedef struct {
	char           *pin;
	tim_polarity_t polarity = TIM_RISING_EDGE;
	uint8_t        prescaler = 0;
	uint8_t        filter = 0;
} tim_config_inputcapture_t;

// TIMER OUTPUT COMPARE MODE ENUM - STRUCT.
typedef enum {
	TIM_FROZEN_MODE = 0x00U,
	TIM_ACTIVE_MODE = 0x01U,
	TIM_INACTIVE_MODE = 0x02U,
	TIM_TOGGLE_MODE = 0x03U,
	TIM_FORCED_ACTIVE_MODE = 0x05U,
	TIM_FORCED_INACTIVE_MODE = 0x04U,
} tim_outputcompare_mode_t;

typedef enum {
	TIM_LEVEL_POLARITY_HIGH,
	TIM_LEVEL_POLARITY_LOW,
} tim_level_polarity_t;

typedef struct {
	char 				 	 *pin;
	tim_outputcompare_mode_t mode = TIM_FROZEN_MODE;
	tim_preload_t 			 preload = TIM_PRELOAD_DISABLE;
	tim_level_polarity_t 	 level_polarity = TIM_LEVEL_POLARITY_HIGH;
} tim_outputcompare_t;


using tim_event_handler_f = std::function<void(tim_channel_t, tim_event_t, void *)>;

class tim{
	public:
		tim(TIM_TypeDef *tim);

		err_t base_initialize(tim_config_t *conf);
		void base_deinitialize(void);
		tim_config_t *get_config(void);

		void register_event_handler(tim_event_handler_f event_handler_function, void *param = NULL);
		void unregister_event_handler(void);

#if PERIPHERAL_DMAC_AVAILABLE
		void link_dmac(dmac_t dmac_udp = NULL, dmac_t dmac_ch1 = NULL, dmac_t dmac_ch2 = NULL, dmac_t dmac_ch3 = NULL, dmac_t dmac_ch4 = NULL);
		void unlink_dmac(void);
		dmac_t get_dmac(uint8_t index = 0);
#endif
		inline void set_prescaler (uint32_t prescaler);
		inline void set_autoreload(uint32_t autoreload);

		inline void reset_counter(void);
		inline uint32_t get_counter(void);

		void delay_us(uint32_t us);
		void delay_ms(uint32_t ms);

/* TIMER base */
		inline void base_start(void);
		inline void base_stop(void);
		inline void base_start_it(void);
		inline void base_stop_it(void);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t base_start_dmac(uint32_t *parrbuff, uint16_t size = 1);
		err_t base_stop_dmac(void);
#endif

/* TIMER PWM output mode */
		err_t pwm_output_initialize(tim_channel_t channel, tim_config_pwm_t *conf);
		inline void pwm_output_set_duty(tim_channel_t channel, uint32_t pwm);

		inline void pwm_output_start(tim_channel_t channel, uint32_t pwm);
		inline void pwm_output_stop(tim_channel_t channel);
		inline void pwm_output_start_it(tim_channel_t channel, uint32_t pwm);
		inline void pwm_output_stop_it(tim_channel_t channel);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t pwm_output_start_dmac(tim_channel_t channel, uint32_t *ppwm_buffer, uint16_t size = 1);
		err_t pwm_output_stop_dmac(tim_channel_t channel);
#endif

/* TIMER PWM input mode */
		err_t pwm_input_initialize(tim_channel_t channel, tim_config_pwm_t *conf);

		err_t pwm_input_start(tim_channel_t channel);
		err_t pwm_input_stop(tim_channel_t channel);
		err_t pwm_input_start_it(tim_channel_t channel);
		err_t pwm_input_stop_it(tim_channel_t channel);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t pwm_input_start_dmac(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size = 1);
		err_t pwm_input_stop_dmac(tim_channel_t channel);
#endif

/* TIMER encoder mode */
		err_t encoder_initialize(tim_config_encoder_t *conf);
		uint32_t encoder_get_base_counter(void);
		int16_t encoder_get_counter(void);

		err_t encoder_start(void);
		err_t encoder_stop(void);
		err_t encoder_start_it(void);
		err_t encoder_stop_it(void);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t encoder_start_dmac(uint32_t *encA_buffer = NULL, uint32_t *encB_buffer = NULL, uint16_t size = 1);
		err_t encoder_stop_dmac(void);
#endif

/* TIMER input capture mode */
		err_t inputcapture_initialize(tim_channel_t channel, tim_config_inputcapture_t *conf);
		uint32_t tim_get_capture_counter(tim_channel_t channel);

		err_t inputcapture_start(tim_channel_t channel);
		err_t inputcapture_stop(tim_channel_t channel);
		err_t inputcapture_start_it(tim_channel_t channel);
		err_t inputcapture_stop_it(tim_channel_t channel);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t inputcapture_start_dmac(tim_channel_t channel, uint32_t *capture_buffer, uint16_t size);
		err_t inputcapture_stop_dmac(tim_channel_t channel);
#endif

/* TIMER output compare mode */
		err_t outputcompare_initialize(tim_channel_t channel, tim_outputcompare_t *conf);
		err_t outputcompare_set_pulse(tim_channel_t channel, uint32_t pulse);

		err_t outputcompare_start(tim_channel_t channel, uint32_t value);
		err_t outputcompare_stop(tim_channel_t channel);
		err_t outputcompare_start_it(tim_channel_t channel, uint32_t value);
		err_t outputcompare_stop_it(tim_channel_t channel);
#if PERIPHERAL_DMAC_AVAILABLE
		err_t outputcompare_start_dmac(tim_channel_t channel, uint32_t *oc_buffer, uint16_t size = 1);
		err_t outputcompare_stop_dmac(tim_channel_t channel);
#endif

	private:
		TIM_TypeDef *_instance;
		tim_config_t *_conf = NULL;
		IRQn_Type _IRQn;
#if defined(STM32F4)
		gpio_function_t _gpio_func;
#endif

		tim_event_handler_f _event_handler = NULL;
		void *_event_parameter = NULL;

		__IO uint32_t _counter = 0;

#if PERIPHERAL_DMAC_AVAILABLE
		dmac_t _dmac_upd = NULL;
		dmac_t _dmac_ch1 = NULL;
		dmac_t _dmac_ch2 = NULL;
		dmac_t _dmac_ch3 = NULL;
		dmac_t _dmac_ch4 = NULL;
		dmac_config_t *_dmac_upd_conf = NULL;
		dmac_config_t *_dmac_ch1_conf = NULL;
		dmac_config_t *_dmac_ch2_conf = NULL;
		dmac_config_t *_dmac_ch3_conf = NULL;
		dmac_config_t *_dmac_ch4_conf = NULL;

		void dmaupd_event_handler(dmac_event_t, void *);
		void dmach1_event_handler(dmac_event_t, void *);
		void dmach2_event_handler(dmac_event_t, void *);
		void dmach3_event_handler(dmac_event_t, void *);
		void dmach4_event_handler(dmac_event_t, void *);
#endif

		void hardware_initialize(void);
		void hardware_deinitialize(void);
		void hardware_output_initialize(void);
		void hardware_input_initialize(void);

		inline void clear_update_isr(void);
		inline void enable_interrupt(void);
		inline void disable_interrupt(void);
};

typedef tim* tim_t;


#if defined(TIM1)
extern tim_t tim1;
void TIM1_CC_IRQHandler(void);
#if !defined(TIM9)
void TIM1_BRK_TIM9_IRQHandler(void);
#endif /* !defined(TIM9) */
#if !defined(TIM10)
void TIM1_UP_TIM10_IRQHandler(void);
#endif /* !defined(TIM10) */
#if !defined(TIM11)
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif /* !defined(TIM11) */
#endif /* defined(TIM1) */

#if defined(TIM2)
extern tim_t tim2;
void TIM2_IRQHandler(void);
#endif

#if defined(TIM3)
extern tim_t tim3;
void TIM3_IRQHandler(void);
#endif

#if defined(TIM4)
extern tim_t tim4;
void TIM4_IRQHandler(void);
#endif

#if defined(TIM5)
extern tim_t tim5;
void TIM5_IRQHandler(void);
#endif

#if defined(TIM6)
extern tim_t tim6;
void TIM6_DAC_IRQHandler(void);
#endif

#if defined(TIM7)
extern tim_t tim7;
void TIM7_IRQHandler(void);
#endif

#if defined(TIM8)
extern tim_t tim8;
void TIM8_CC_IRQHandler(void);
#if !defined(TIM12)
void TIM8_BRK_TIM12_IRQHandler(void);
#endif /* !defined(TIM12) */
#if !defined(TIM13)
void TIM8_UP_TIM13_IRQHandler(void);
#endif /* !defined(TIM13) */
#if !defined(TIM14)
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif /* !defined(TIM14) */
#endif

#if defined(TIM9)
extern tim_t tim9;
void TIM1_BRK_TIM9_IRQHandler(void);
#endif

#if defined(TIM10)
extern tim_t tim10;
void TIM1_UP_TIM10_IRQHandler(void);
#endif

#if defined(TIM11)
extern tim_t tim11;
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif

#if defined(TIM12)
extern tim_t tim12;
void TIM8_BRK_TIM12_IRQHandler(void);
#endif

#if defined(TIM13)
extern tim_t tim13;
void TIM8_UP_TIM13_IRQHandler(void);
#endif

#if defined(TIM14)
extern tim_t tim14;
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_TIM_AVAILABLE 0
#endif /* CONFIG_PERIPH_TIM_EN */

#endif /* PERIPHERALS_TIM_H_ */

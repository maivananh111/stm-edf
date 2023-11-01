/*
 *  dmac.h
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_DMAC_H_
#define PERIPHERALS_DMAC_H_


#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_DMAC_EN && (defined(DMA1) || defined(DMA2) || defined(DMA3))

#define PERIPHERAL_DMAC_AVAILABLE 1

#include "iostream"
#include "functional"

#include "common/error_check.h"
#include "common/macro.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


#ifdef __cplusplus
extern "C"{
#endif

#if defined(STM32F1)
#if defined(DMA1)
#define DMA1_Stream1 DMA1_Channel1
#define DMA1_Stream2 DMA1_Channel2
#define DMA1_Stream3 DMA1_Channel3
#define DMA1_Stream4 DMA1_Channel4
#define DMA1_Stream5 DMA1_Channel5
#define DMA1_Stream6 DMA1_Channel6
#define DMA1_Stream7 DMA1_Channel7
#endif
#if defined(DMA2)
#define DMA2_Stream1 DMA2_Channel1
#define DMA2_Stream2 DMA2_Channel2
#define DMA2_Stream3 DMA2_Channel3
#define DMA2_Stream4 DMA2_Channel4
#define DMA2_Stream5 DMA2_Channel5
#define DMA2_Stream6 DMA2_Channel6
#define DMA2_Stream7 DMA2_Channel7
#endif

typedef DMA_Channel_TypeDef DMA_Stream_TypeDef;
#endif

typedef enum{
#if defined(STM32F4)
	DMAC_CHANNEL0,
#endif /* STM32F1 */
	DMAC_CHANNEL1,
	DMAC_CHANNEL2,
	DMAC_CHANNEL3,
	DMAC_CHANNEL4,
	DMAC_CHANNEL5,
	DMAC_CHANNEL6,
	DMAC_CHANNEL7,
}dmac_channel_t;

typedef enum{
	DMAC_PERIPH_TO_MEM,
	DMAC_MEM_TO_PERIPH,
	DMAC_MEM_TO_MEM,
}dmac_direction_t;

typedef enum{
	DMAC_MODE_ONCE,
	DMAC_MODE_CIRCULAR,
} dmac_mode_t;

typedef enum{
	DMAC_DATASIZE_8BIT,
	DMAC_DATASIZE_16BIT,
	DMAC_DATASIZE_32BIT,
} dmac_datasize_t;

#if defined(STM32F4)
typedef enum{
	DMAC_NOFIFO = (0U),
	DMAC_FIF0 = DMA_SxFCR_DMDIS,
}dmac_fifo_t;

typedef enum{
	DMAC_SINGLE_TRANFER = (0U),
	DMAC_BURST_4INCREMENTAL,
	DMAC_BURST_8INCREMENTAL,
	DMAC_BURST_16INCREMENTAL,
} dmac_burst_t;
#endif /* STM32F4 */

typedef enum{
	DMAC_CHANNEL_PRIORITY_LOW = (0U),
	DMAC_CHANNEL_PRIORITY_MEDIUM,
	DMAC_CHANNEL_PRIORITY_HIGH,
	DMAC_CHANNEL_PRIORITY_VERYHIGH,
} dmac_channelpriority_t;

#if defined(STM32F1)
typedef enum{
	DMAC_TRANSFER_COMPLETE_INTERRUPT = DMA_CCR_TCIE,
	DMAC_HALF_TRANSFER_INTERRUPT     = DMA_CCR_HTIE,
	DMAC_TRANSFER_ERROR_INTERRUPT    = DMA_CCR_TEIE,
} dmac_interruptoption_t;
/** -------------------------------------------------------------------------------------------------- **/
#elif defined(STM32F4)
typedef enum{
	DMAC_TRANSFER_COMPLETE_INTERRUPT = DMA_SxCR_TCIE,
	DMAC_HALF_TRANSFER_INTERRUPT = DMA_SxCR_HTIE,
	DMAC_TRANSFER_ERROR_INTERRUPT = DMA_SxCR_TEIE,
	DMAC_DIRECTMODE_ERROR_INTERRUPT = DMA_SxCR_DMEIE,
} dmac_interruptoption_t;
#endif /* STM32F4 */

typedef enum{
	DMAC_EVENT_NOEVENT,
	DMAC_EVENT_TRANFER_COMPLETE,
	DMAC_EVENT_HALF_TRANFER,
	DMAC_EVENT_TRANFER_ERROR,
} dmac_event_t;

typedef struct {
	DMA_Stream_TypeDef 	          *stream = DMA1_Stream1;
	dmac_channel_t 		          channel = DMAC_CHANNEL1;
	dmac_direction_t 	        direction = DMAC_MEM_TO_PERIPH;
	dmac_mode_t 	 		             mode = DMAC_MODE_ONCE;
	dmac_datasize_t 		         datasize = DMAC_DATASIZE_8BIT;
#if defined(STM32F4)
	dmac_fifo_t 		                 fifo = DMAC_NOFIFO;
	dmac_burst_t 		            burst = DMAC_BURST_4INCREMENTAL;
#endif /* STM32F4 */
	uint32_t 			  interruptoption = DMAC_TRANSFER_COMPLETE_INTERRUPT | DMAC_HALF_TRANSFER_INTERRUPT | DMAC_TRANSFER_ERROR_INTERRUPT;
	dmac_channelpriority_t channelpriority = DMAC_CHANNEL_PRIORITY_HIGH;
	uint32_t 			interruptpriority = 0;
} dmac_config_t;

struct dmac_session_config_t{
	uint32_t      psource = 0UL;
	uint32_t        pdest = 0UL;
	uint16_t     xfersize = 0U;
	bool interrupt_enable = true;
	uint32_t      timeout = CONFIG_PERIPH_DMAC_DEFAULT_OPERATION_TIMEOUT;
};

typedef void(*dmac_evcb_t)(dmac_event_t, void *);

class dmac{
	public:
	/** Constructor */
		dmac(DMA_TypeDef *dmac);
	/** Initialize/ deinitialize */
		err_t initialize(dmac_config_t *conf);
		void deinitialize(void);
	/** Event handler */
		void register_event_handler(std::function<void(dmac_event_t event, void *param)> event_handler_function, void *event_parameter = NULL);
		void unregister_event_handler(void);
		void event_handle(dmac_event_t event);
	/** Get parameter */
		DMA_TypeDef *get_instance(void);
		dmac_config_t *get_config(void);
		uint16_t get_transfer_counter(void);
		SemaphoreHandle_t get_mutex(void);
	/** Configure and start transfer session */
		err_t config_start_session(struct dmac_session_config_t conf);
	/** Start/stop */
		void stop_session(void);
	/** Transfer session wait */
		err_t wait_for(dmac_event_t break_event, uint32_t timeout);

	private:
		DMA_TypeDef *_instance;
		dmac_config_t *_conf;
		IRQn_Type _IRQn;

		std::function<void(dmac_event_t, void *)> _event_handler;
		void *_event_parameter;

		SemaphoreHandle_t _mutex;

#if defined(STM32F4)
		__IO uint32_t _int_index = 0U;
		uint32_t _stream = 0U;
		uint8_t _dma_reg_level = 0U; // 0 is LOW register.
		volatile uint32_t *_ICFR = 0x00000000U;
		volatile uint32_t *_ISR  = 0x00000000U;
#endif /* STM32F4 */

		void clear_IFCR(__IO uint32_t Value);
		void clear_all_flag(void);
		__IO uint32_t get_ISR(void);
		void config(uint32_t psource, uint32_t pdest, uint32_t size);
};

typedef dmac* dmac_t;

#if defined(DMA1_Stream0)
extern dmac_t dma1_stream0;
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
extern dmac_t dma1_stream1;
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
extern dmac_t dma1_stream2;
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
extern dmac_t dma1_stream3;
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
extern dmac_t dma1_stream4;
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
extern dmac_t dma1_stream5;
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
extern dmac_t dma1_stream6;
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
extern dmac_t dma1_stream7;
#endif /* defined(DMA1_Stream7) */
#if defined(DMA2_Stream0)
extern dmac_t dma2_stream0;
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
extern dmac_t dma2_stream1;
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
extern dmac_t dma2_stream2;
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
extern dmac_t dma2_stream3;
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
extern dmac_t dma2_stream4;
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
extern dmac_t dma2_stream5;
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
extern dmac_t dma2_stream6;
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
extern dmac_t dma2_stream7;
#endif /* defined(DMA2_Stream7) */

/**
 *  DMA1 stream interrupt request handler prototype.
 */
#if defined(STM32F1)
/* DMA1 IRQ HANDLER */
#if defined(DMA1_Channel1)
void DMA1_Channel1_IRQHandler(void);
#endif /* defined(DMA1_Channel1) */
#if defined(DMA1_Channel2)
void DMA1_Channel2_IRQHandler(void);
#endif /* defined(DMA1_Channel2) */
#if defined(DMA1_Channel3)
void DMA1_Channel3_IRQHandler(void);
#endif /* defined(DMA1_Channel3) */
#if defined(DMA1_Channel4)
void DMA1_Channel4_IRQHandler(void);
#endif /* defined(DMA1_Channel4) */
#if defined(DMA1_Channel5)
void DMA1_Channel5_IRQHandler(void);
#endif /* defined(DMA1_Channel5) */
#if defined(DMA1_Channel6)
void DMA1_Channel6_IRQHandler(void);
#endif /* defined(DMA1_Channel6) */
#if defined(DMA1_Channel7)
void DMA1_Channel7_IRQHandler(void);
#endif /* defined(DMA1_Channel7) */

/**
 *  DMA2 channel interrupt request handler prototype.
 */
#if defined(DMA2_Channel1)
void DMA2_Channel1_IRQHandler(void);
#endif /* defined(DMA2_Channel1) */
#if defined(DMA2_Channel2)
void DMA2_Channel2_IRQHandler(void);
#endif /* defined(DMA2_Channel2) */
#if defined(DMA2_Channel3)
void DMA2_Channel3_IRQHandler(void);
#endif /* defined(DMA2_Channel3) */
#if defined(DMA2_Channel4)
void DMA2_Channel4_IRQHandler(void);
#endif /* defined(DMA2_Channel4) */
#if defined(DMA2_Channel5)
void DMA2_Channel5_IRQHandler(void);
#endif /* defined(DMA2_Channel5) */
#if defined(DMA2_Channel6)
void DMA2_Channel6_IRQHandler(void);
#endif /* defined(DMA2_Channel6) */
#if defined(DMA2_Channel7)
void DMA2_Channel7_IRQHandler(void);
#endif /* defined(DMA2_Channel7) */

#endif /* STM32F1 */



/**
 *  DMA1 Stream interrupt request handler prototype.
 */
#if defined(STM32F4)
/* DMA1 IRQ HANDLER */
#if defined(DMA1_Stream0)
void DMA1_Stream0_IRQHandler(void);
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
void DMA1_Stream1_IRQHandler(void);
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
void DMA1_Stream2_IRQHandler(void);
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
void DMA1_Stream3_IRQHandler(void);
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
void DMA1_Stream4_IRQHandler(void);
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
void DMA1_Stream5_IRQHandler(void);
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
void DMA1_Stream6_IRQHandler(void);
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
void DMA1_Stream7_IRQHandler(void);
#endif /* defined(DMA1_Stream7) */

/**
 *  DMA2 Stream interrupt request handler prototype.
 */
#if defined(DMA2_Stream0)
void DMA2_Stream0_IRQHandler(void);
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
void DMA2_Stream1_IRQHandler(void);
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
void DMA2_Stream2_IRQHandler(void);
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
void DMA2_Stream3_IRQHandler(void);
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
void DMA2_Stream4_IRQHandler(void);
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
void DMA2_Stream5_IRQHandler(void);
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
void DMA2_Stream6_IRQHandler(void);
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
void DMA2_Stream7_IRQHandler(void);
#endif /* defined(DMA2_Stream7) */

#endif /* STM32F4 */

#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_DMAC_AVAILABLE 0
#endif /* CONFIG_PERIPH_DMAC_EN && (defined(DMA1) || defined(DMA2) || defined(DMA3)) */

#endif /* PERIPHERALS_DMAC_H_ */

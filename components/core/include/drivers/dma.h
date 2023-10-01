/*
 *  dma.h
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_DMA_H_
#define PERIPHERALS_DMA_H_


#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_DMA_ENABLE && (defined(DMA1) || defined(DMA2) || defined(DMA3))

#define PERIPHERAL_DMA_AVAILABLE 1

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
	DMA_CHANNEL0,
#endif /* STM32F1 */
	DMA_CHANNEL1,
	DMA_CHANNEL2,
	DMA_CHANNEL3,
	DMA_CHANNEL4,
	DMA_CHANNEL5,
	DMA_CHANNEL6,
	DMA_CHANNEL7,
}dma_channel_t;

typedef enum{
	DMA_PERIPH_TO_MEM,
	DMA_MEM_TO_PERIPH,
	DMA_MEM_TO_MEM,
}dma_direction_t;

typedef enum{
	DMA_MODE_ONCE,
	DMA_MODE_CIRCULAR,
} dma_mode_t;

typedef enum{
	DMA_DATASIZE_8BIT,
	DMA_DATASIZE_16BIT,
	DMA_DATASIZE_32BIT,
} dma_datasize_t;

#if defined(STM32F4)
typedef enum{
	DMA_NOFIFO = (0U),
	DMA_FIF0 = DMA_SxFCR_DMDIS,
}dma_fifo_t;

typedef enum{
	DMA_SINGLE_TRANFER = (0U),
	DMA_BURST_4INCREMENTAL,
	DMA_BURST_8INCREMENTAL,
	DMA_BURST_16INCREMENTAL,
} dma_burst_t;
#endif /* STM32F4 */

typedef enum{
	DMA_CHANNEL_PRIORITY_LOW = (0U),
	DMA_CHANNEL_PRIORITY_MEDIUM,
	DMA_CHANNEL_PRIORITY_HIGH,
	DMA_CHANNEL_PRIORITY_VERYHIGH,
} dma_channelpriority_t;

#if defined(STM32F1)
typedef enum{
	DMA_TRANSFER_COMPLETE_INTERRUPT = DMA_CCR_TCIE,
	DMA_HALF_TRANSFER_INTERRUPT     = DMA_CCR_HTIE,
	DMA_TRANSFER_ERROR_INTERRUPT    = DMA_CCR_TEIE,
} dma_interruptoption_t;
/** -------------------------------------------------------------------------------------------------- **/
#elif defined(STM32F4)
typedef enum{
	DMA_TRANSFER_COMPLETE_INTERRUPT = DMA_SxCR_TCIE,
	DMA_HALF_TRANSFER_INTERRUPT = DMA_SxCR_HTIE,
	DMA_TRANSFER_ERROR_INTERRUPT = DMA_SxCR_TEIE,
	DMA_DIRECTMODE_ERROR_INTERRUPT = DMA_SxCR_DMEIE,
} dma_interruptoption_t;
#endif /* STM32F4 */

typedef enum{
	DMA_EVENT_NOEVENT,
	DMA_EVENT_TRANFER_COMPLETE,
	DMA_EVENT_HALF_TRANFER,
	DMA_EVENT_TRANFER_ERROR,
} dma_event_t;

typedef struct {
	DMA_Stream_TypeDef 	          *stream = DMA1_Stream1;
	dma_channel_t 		          channel = DMA_CHANNEL1;
	dma_direction_t 	        direction = DMA_MEM_TO_PERIPH;
	dma_mode_t 	 		             mode = DMA_MODE_ONCE;
	dma_datasize_t 		         datasize = DMA_DATASIZE_8BIT;
#if defined(STM32F4)
	dma_fifo_t 		                 fifo = DMA_NOFIFO;
	dma_burst_t 		            burst = DMA_BURST_4INCREMENTAL;
#endif /* STM32F4 */
	uint32_t 			  interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT | DMA_HALF_TRANSFER_INTERRUPT | DMA_TRANSFER_ERROR_INTERRUPT;
	dma_channelpriority_t channelpriority = DMA_CHANNEL_PRIORITY_HIGH;
	uint32_t 			interruptpriority = 0;
} dma_config_t;

struct dma_session_config_t{
	uint32_t      psource = 0UL;
	uint32_t        pdest = 0UL;
	uint16_t     xfersize = 0U;
	bool interrupt_enable = true;
	uint32_t      timeout = CONFIG_PERIPH_DMA_DEFAULT_OPERATION_TIMEOUT;
};

typedef void(*dma_evcb_t)(dma_event_t, void *);

class dma{
	public:
	/** Constructor */
		dma(DMA_TypeDef *dma);
	/** Initialize/ deinitialize */
		err_t initialize(dma_config_t *conf);
		void deinitialize(void);
	/** Event handler */
		void register_event_handler(std::function<void(dma_event_t event, void *param)> event_handler_function, void *event_parameter = NULL);
		void unregister_event_handler(void);
		void event_handle(dma_event_t event);
	/** Get parameter */
		DMA_TypeDef *get_instance(void);
		dma_config_t *get_config(void);
		uint16_t get_transfer_counter(void);
		SemaphoreHandle_t get_mutex(void);
	/** Configure and start transfer session */
		err_t config_start_session(struct dma_session_config_t conf);
	/** Start/stop */
		void stop_session(void);
	/** Transfer session wait */
		err_t wait_for(dma_event_t break_event, uint32_t timeout);

	private:
		DMA_TypeDef *_instance;
		dma_config_t *_conf;
		IRQn_Type _IRQn;

		std::function<void(dma_event_t, void *)> _event_handler;
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

typedef dma* dma_t;

#if defined(DMA1_Stream0)
extern dma_t dma1_stream0;
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
extern dma_t dma1_stream1;
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
extern dma_t dma1_stream2;
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
extern dma_t dma1_stream3;
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
extern dma_t dma1_stream4;
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
extern dma_t dma1_stream5;
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
extern dma_t dma1_stream6;
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
extern dma_t dma1_stream7;
#endif /* defined(DMA1_Stream7) */
#if defined(DMA2_Stream0)
extern dma_t dma2_stream0;
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
extern dma_t dma2_stream1;
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
extern dma_t dma2_stream2;
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
extern dma_t dma2_stream3;
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
extern dma_t dma2_stream4;
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
extern dma_t dma2_stream5;
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
extern dma_t dma2_stream6;
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
extern dma_t dma2_stream7;
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
#define PERIPHERAL_DMA_AVAILABLE 0
#endif /* CONFIG_PERIPH_DMA_ENABLE && (defined(DMA1) || defined(DMA2) || defined(DMA3)) */

#endif /* PERIPHERALS_DMA_H_ */

/*
 * uart.h
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#ifndef PERIPHERALS_UART_H_
#define PERIPHERALS_UART_H_


#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_UART_EN && (defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) || defined(UART7) || defined(UART8))
#define PERIPHERAL_UART_AVAILABLE 1

#include "stdio.h"
#include "iostream"
#include "functional"

#include "common/error_check.h"
#include "drivers/gpio.h"
#if CONFIG_PERIPH_DMAC_EN
#include "drivers/dmac.h"
#endif /* CONFIG_PERIPH_DMAC_EN */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
	UART_EVENT_NOEVENT,
	UART_EVENT_TRANSMIT_COMPLETE,
	UART_EVENT_RECEIVE_COMPLETE,
	UART_EVENT_IDLE,
	UART_EVENT_ENDCHAR,
	UART_EVENT_ERROR,
} uart_event_t;

typedef enum{
	UART_RECEPTION_UNTIL_COMPLETED,
	UART_RECEPTION_UNTIL_ENDCHAR,
	UART_RECEPTION_UNTIL_IDLE,
} uart_reception_t;

typedef struct{
	uint32_t          baudrate = 115200U;
	uint32_t interruptpriority = 0U;
	char     			*txpin = NULL;
	char     			*rxpin = NULL;
} uart_config_t;

struct uart_xfer_t{
	uint8_t *buffer = NULL;
	uint16_t length = 0U;
	uint16_t  count = 0U;
};

typedef void (*uart_evcb_t)(uart_event_t, void *);

class uart;
typedef uart* uart_t;

#define UART_CONFIG_DEFAULT() 						\
{							  						\
	.baudrate = CONFIG_PERIPH_UART_DEFAULT_BAUDRATE,\
	.interruptpriority = 0U,						\
}

class uart {
	public:
	/** Constructor */
		uart(USART_TypeDef *uart);
	/** Initialize/deinitialize */
		err_t initialize(uart_config_t *conf);
		err_t deinitialize(void);
	/** Event handle */
		void register_event_handler(std::function<void(uart_event_t event, void *param)> event_handler_function, void *param = NULL);
		void unregister_event_handler(void);
		void event_handle(uart_event_t event);
	/** DMA link */
#if PERIPHERAL_DMAC_AVAILABLE
		void link_dma(dmac_t txdma = NULL, dmac_t rxdma = NULL);
		void unlink_dma(void);
#endif /* PERIPHERAL_DMAC_AVAILABLE */
	/** Get parameter */
		USART_TypeDef *get_instance(void);
		uart_config_t *get_config(void);
		IRQn_Type get_irq(void);
		uart_reception_t get_reception_mode(void);
		char get_endchar(void);
#if PERIPHERAL_DMAC_AVAILABLE
		dmac_t get_txdma(void);
		dmac_t get_rxdma(void);
#endif /* PERIPHERAL_DMAC_AVAILABLE */
		SemaphoreHandle_t get_txmutex(void);
		SemaphoreHandle_t get_rxmutex(void);
		err_t get_rxbuffer(uint8_t **ppdata);
		uint16_t get_rxbuffersize(void);
	/** Operation mode configure */
		void set_endchar(char endchar = '\0');
		err_t reload_rxbuffer(uint16_t new_buffer_size = 0U);
		void release_rxbuffer(void);

	/** UART operation */
		err_t transmit(uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);
		err_t receive(uart_reception_t reception, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);
		err_t transmit_it(uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_it(uart_reception_t reception, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);
		err_t transmit_dma(uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_dma(uart_reception_t reception, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_UART_DEFAULT_OPERATION_TIMEOUT);

		void abort_transmit_it(void);
		void abort_receive_it(void);
		void abort_all_it(void);

		err_t abort_transmit_dma(void);
		err_t abort_receive_dma(void);
		err_t abort_all_dma(void);

#if PERIPHERAL_DMAC_AVAILABLE
		err_t txdma_stop(void);
		err_t rxdma_stop(void);

		void txdma_event_handler(dmac_event_t event, void *param);
		void rxdma_event_handler(dmac_event_t event, void *param);
#endif /* PERIPHERAL_DMAC_AVAILABLE */

		uart_xfer_t txinfo, rxinfo;

	private:
		USART_TypeDef *_instance;
		uart_config_t *_conf;
		gpio_pin_t _txpin, _rxpin;
		IRQn_Type _IRQn;
		uint32_t _bus_freq_hz;

		SemaphoreHandle_t _txmutex, _rxmutex;

#if PERIPHERAL_DMAC_AVAILABLE
		dmac_t _txdma = NULL, _rxdma = NULL;
		dmac_config_t *_txdma_conf = NULL, *_rxdma_conf = NULL;
#endif /* PERIPHERAL_DMAC_AVAILABLE */

		void *_event_parameter = NULL;
		std::function<void(uart_event_t event, void *param)> _event_handler;
		char _endchar = '\0';
		uart_reception_t _reception_mode = UART_RECEPTION_UNTIL_COMPLETED;

		err_t hardware_initialize(void);
		void hardware_deinitialize(void);
};


uart_t uart_str_decode(char *str);


#if defined(USART1)
extern uart_t uart1;
void USART1_IRQHandler(void);
#endif
#if defined(USART2)
extern uart_t uart2;
void USART2_IRQHandler(void);
#endif
#if defined(USART3)
extern uart_t uart3;
void USART3_IRQHandler(void);
#endif
#if defined(UART4)
extern uart_t uart4;
void UART4_IRQHandler(void);
#endif
#if defined(UART5)
extern uart_t uart5;
void UART5_IRQHandler(void);
#endif
#if defined(USART6)
extern uart_t uart6;
void USART6_IRQHandler(void);
#endif
#if defined(UART7)
extern uart_t uart7;
void UART7_IRQHandler(void);
#endif
#if defined(UART8)
extern uart_t uart8;
void UART8_IRQHandler(void);
#endif


#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_UART_AVAILABLE 0
#endif /* CONFIG_PERIPH_UART_EN && (defined(USART1) || defined(USART2) || defined(USART3) || defined(UUART4) || defined(UART5) || defined(USART6) || defined(UART7) || defined(UART8)) */

#endif /* PERIPHERALS_UART_H_ */


/*
 * spi.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_SPI_H_
#define PERIPHERALS_SPI_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_SPI_ENABLE && (defined(SPI1) || defined(SPI2) || defined(SPI3) || defined(SPI4) || defined(SPI5) || defined(SPI6))
#define PERIPHERAL_SPI_AVAILABLE 1

#include "stdint.h"
#include "iostream"
#include "functional"

#include "common/error_check.h"
#include "drivers/gpio.h"
#if CONFIG_PERIPH_DMA_ENABLE
#include "drivers/dma.h"
#endif /* ENABLE_DMA */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */


typedef enum{
	SPI_FULLDUPLEX_MASTER = 0x01U,
	SPI_HALFDUPLEX_MASTER = 0x02U,

	SPI_FULLDUPLEX_SLAVE  = 0x04U,
	SPI_HALFDUPLEX_SLAVE  = 0x08U,
} spi_controler_t;

typedef enum{
	SPI_DATASIZE_8BIT = 0U,
	SPI_DATASIZE_16BIT,
} spi_datasize_t;

typedef enum{
	SPI_BITORDER_MSB = 0U,
	SPI_BITORDER_LSB,
} spi_bitorder_t;

typedef enum{
	SPI_CLOCKDIVISION_2 = 0UL,
	SPI_CLOCKDIVISION_4,
	SPI_CLOCKDIVISION_8,
	SPI_CLOCKDIVISION_16,
	SPI_CLOCKDIVISION_32,
	SPI_CLOCKDIVISION_64,
	SPI_CLOCKDIVISION_128,
	SPI_CLOCKDIVISION_256,
} spi_clockdivision_t;

typedef enum{
	SPI_CLOCK_CPOL0_CPHA0 = 0U,
	SPI_CLOCK_CPOL0_CPHA1,
	SPI_CLOCK_CPOL1_CPHA0,
	SPI_CLOCK_CPOL1_CPHA1,
} spi_clocksample_t;

typedef enum {
	SPI_SOFTWARE_NSS,
	SPI_HARDWARE_NSS,
} spi_nss_t;

#if defined(STM32F1)
typedef enum {
	SPI_NSS_INPUT,
	SPI_NSS_OUTPUT,
} spi_nss_direction_t;
#endif /* STM32F1 */

typedef enum{
	SPI_EVENT_NOEVENT           = 0x00,
	SPI_EVENT_TRANSMIT_COMPLETE = 0x01,
	SPI_EVENT_RECEIVE_COMPLETE  = 0x02,
	SPI_EVENT_ERROR             = 0x04,
} spi_event_t;

typedef void (*spi_evcb_t)(spi_event_t, void *);

typedef struct {
	spi_controler_t 	  controller        = SPI_FULLDUPLEX_MASTER;
	spi_datasize_t 		  datasize          = SPI_DATASIZE_8BIT;
	spi_bitorder_t   	  bitorder  	    = SPI_BITORDER_MSB;
	spi_clockdivision_t   clockdivision     = SPI_CLOCKDIVISION_4;
	spi_clocksample_t 	  clocksample 	    = SPI_CLOCK_CPOL0_CPHA0;
	uint32_t 			  interruptpriority = 0;
	spi_nss_t             nss 				= SPI_SOFTWARE_NSS;
#if defined(STM32F1)
	spi_nss_direction_t   nss_dir           = SPI_NSS_INPUT;
#endif /* STM32F1 */
	char *clkpin;
	char *misopin = NULL;
	char *mosipin = NULL;
	char *nsspin = NULL;
} spi_config_t;

struct spi_xfer_t{
	uint32_t buffer = 0U;
	uint32_t count = 0U;
	uint16_t length = 0U;
};

typedef void (*spi_evcb_t)(spi_event_t, void *);

class spi;
typedef spi* spi_t;

class spi{
	public:
	/** Constructor. */
		spi(SPI_TypeDef *spi);
	/** Initialize and deinitilize function. */
		err_t initialize(spi_config_t *conf);
		void deinitilize(void);
	/** Event handle. */
		void register_event_handler(std::function<void(spi_event_t event, void *param)> event_handler_function, void *param = NULL);
		void unregister_event_handler(void);
		void event_handle(spi_event_t event);
	/** DMA declare */
#if PERIPHERAL_DMA_AVAILABLE
		void install_dma(dma_t txdma = NULL, dma_config_t *txdma_conf = NULL, dma_t rxdma = NULL, dma_config_t *rxdma_conf = NULL);
		void uninstall_dma(void);
#endif /* PERIPHERAL_DMA_AVAILABLE */
	/** Get parameter. */
		SPI_TypeDef *get_instance(void);
		spi_config_t *get_config(void);
		IRQn_Type get_irq(void);
#if PERIPHERAL_DMA_AVAILABLE
		dma_t get_txdma(void);
		dma_t get_rxdma(void);
#endif /* PERIPHERAL_DMA_AVAILABLE */
		SemaphoreHandle_t get_txmutex(void);
		SemaphoreHandle_t get_rxmutex(void);
	/** SPI operation. */
		err_t transmit(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t receive(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t transmit_receive(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);

		err_t transmit_it(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_it(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t transmit_receive_it(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);

		err_t transmit_dma(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_dma(uint32_t pdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);
		err_t transmit_receive_dma(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout = CONFIG_PERIPH_SPI_DEFAULT_OPERATION_TIMEOUT);

		err_t abort_transmit_it(void);
		err_t abort_receive_it(void);
		err_t abort_all_it(void);
#if PERIPHERAL_DMA_AVAILABLE
		err_t abort_transmit_dma(void);
		err_t abort_receive_dma(void);
		err_t abort_all_dma(void);

		err_t txdma_stop(void);
		err_t rxdma_stop(void);

		void txdma_event_handler(dma_event_t event, void *param);
		void rxdma_event_handler(dma_event_t event, void *param);
#endif /* PERIPHERAL_DMA_AVAILABLE */

		spi_xfer_t txinfo, rxinfo;

	private:
		spi_config_t *_conf = NULL;
		SPI_TypeDef *_instance;
		gpio_pin_t _clkpin, _misopin, _mosipin, _nsspin;
		IRQn_Type _IRQn;

#if PERIPHERAL_DMA_AVAILABLE
		dma_t _txdma = NULL, _rxdma = NULL;
		dma_config_t *_txdma_conf = NULL, *_rxdma_conf = NULL;
#endif /* PERIPHERAL_DMA_AVAILABLE */

		void *_event_parameter = NULL;
		std::function<void(spi_event_t event, void *param)> _event_handler;

		SemaphoreHandle_t _txmutex;
		SemaphoreHandle_t _rxmutex;

		err_t hardware_initialize(void);
		void hardware_deinitialize(void);
};

spi_t spi_str_decode(char *str);

#if defined(SPI1)
extern spi_t spi1;
void SPI1_IRQHandler(void);
#endif /* defined(SPI1) */
#if defined(SPI2)
extern spi_t spi2;
void SPI2_IRQHandler(void);
#endif /* defined(SPI2) */
#if defined(SPI3)
extern spi_t spi3;
void SPI3_IRQHandler(void);
#endif /* defined(SPI3) */
#if defined(SPI4)
extern spi_t spi4;
void SPI4_IRQHandler(void);
#endif /* defined(SPI4) */
#if defined(SPI5)
extern spi_t spi5;
void SPI5_IRQHandler(void);
#endif /* defined(SPI5) */
#if defined(SPI6)
extern spi_t spi6;
void SPI6_IRQHandler(void);
#endif /* defined(SPI6) */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#else
#define PERIPHERAL_SPI_AVAILABLE 0
#endif /* CONFIG_PERIPH_SPI_ENABLE && (defined(SPI1) || defined(SPI2) || defined(SPI3) || defined(SPI4) || defined(SPI5) || defined(SPI6)) */

#endif /* PERIPHERALS_SPI_H_ */

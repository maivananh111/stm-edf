/*
 * spi.cpp
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */
#include "drivers/spi.h"
#if PERIPHERAL_SPI_AVAILABLE

#include "stdlib.h"
#include "string.h"
#include "common/bits_check.h"
#include "common/expression_check.h"
#include "common/macro.h"
#include "drivers/system.h"
#include "drivers/rcc.h"
#include "common/log_monitor.h"



#define SPI_TIMEOUT 1000U

static const char *TAG = "SPI";

#ifdef CONFIG_PERIPH_SPI_LOG
#define SPI_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, TAG, __MSG__);
#else
#define SPI_DBG(__MSG__)\
{}
#endif


static void SPI_TXE_IRQHandler(spi_t pspi);
static void SPI_RXNE_IRQHandler(spi_t pspi);
static void SPI_ERR_IRQHandler(spi_t pspi);
static void SPICommon_IRQHandler(spi_t pspi);



spi_t spi_str_decode(char *str){
#if defined(SPI1)
	if(strcasecmp(str, "SPI1") == 0)
		return spi1;
#endif /* defined(SPI1) */
#if defined(SPI2)
	if(strcasecmp(str, "SPI2") == 0)
		return spi2;
#endif /* defined(SPI2) */
#if defined(SPI3)
	if(strcasecmp(str, "SPI3") == 0)
		return spi3;
#endif /* defined(SPI3) */
#if defined(SPI4)
	if(strcasecmp(str, "SPI4") == 0)
		return spi4;
#endif /* defined(SPI1) */
#if defined(SPI5)
	if(strcasecmp(str, "SPI5") == 0)
		return spi5;
#endif /* defined(SPI2) */
#if defined(SPI6)
	if(strcasecmp(str, "SPI6") == 0)
		return spi6;
#endif /* defined(SPI3) */

	return spi1;
}


spi::spi(SPI_TypeDef *Spi){
	_instance = Spi;
}

err_t spi::hardware_initialize(void){
	err_t ret;

#if defined(SPI1)
	if (_instance == SPI1){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
		_IRQn = SPI1_IRQn;
	}
#endif /* defined(SPI1) */
#if defined(SPI2)
	if(_instance == SPI2){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
		_IRQn = SPI2_IRQn;
	}
#endif /* defined(SPI2) */
#if defined(SPI3)
	if(_instance == SPI3){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);
		_IRQn = SPI3_IRQn;
	}
	if(_instance == SPI3)
#endif /* defined(SPI3) */
#if defined(SPI4)
	if(_instance == SPI4){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI4EN);
		_IRQn = SPI4_IRQn;
	}
#endif /* defined(SPI4) */
#if defined(SPI5)
	if(_instance == SPI5){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI5EN);
		_IRQn = SPI5_IRQn;
	}
#endif /* defined(SPI6) */
#if defined(SPI6)
	if(_instance == SPI6){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI6EN);
		_IRQn = SPI6_IRQn;
	}
#endif /* defined(SPI4) */

	if(gpio_str_decode(_conf->clkpin,  &_clkpin) == false)  goto error;
	if(gpio_str_decode(_conf->misopin, &_misopin) == false) goto error;
	if(gpio_str_decode(_conf->mosipin, &_mosipin) == false) goto error;
	if(gpio_str_decode(_conf->nsspin,  &_nsspin) == false)  goto error;
	goto pass;

	error:
	ERROR_SET(ret, E_INVALID);
	SPI_DBG("SPI pin invalid");
	return ret;

	pass:
#if defined(STM32F1)
	if(_conf->controller == SPI_FULLDUPLEX_MASTER || _conf->controller == SPI_HALFDUPLEX_MASTER){
		gpio_set_direction(_clkpin.port, _clkpin.pinnum, GPIO_FUNCTION);
		gpio_set_direction(_mosipin.port, _mosipin.pinnum, GPIO_FUNCTION);

		if(_conf->controller == SPI_FULLDUPLEX_MASTER)
			gpio_set_direction(_misopin.port, _misopin.pinnum, GPIO_INPUT);
		if(_conf->nss == SPI_HARDWARE_NSS){
			if(_conf->nss_dir == SPI_NSS_INPUT)
				gpio_set_direction(_nsspin.port, _nsspin.pinnum, GPIO_INPUT);
			else
				gpio_set_direction(_nsspin.port, _nsspin.pinnum, GPIO_FUNCTION);
		}
	}
	else{
		gpio_set_direction(_clkpin.port, _clkpin.pinnum, GPIO_INPUT);
		gpio_set_direction(_misopin.port, _misopin.pinnum, GPIO_FUNCTION);
		if(_conf->controller == SPI_FULLDUPLEX_SLAVE)
			gpio_set_direction(_mosipin.port, _mosipin.pinnum, GPIO_INPUT);
		if(_conf->nss == SPI_HARDWARE_NSS)
			gpio_set_direction(_nsspin.port, _nsspin.pinnum, GPIO_INPUT);
	}

#elif defined(STM32F4)
	gpio_function_t function;
#if defined(SPI4) || defined(SPI5) || defined(SPI6)
	function = AF5_SPI1_6;
#else
	if(_instance == SPI1 || _instance == SPI2)
		function = AF5_SPI1_2;
	else
		function = AF6_SPI3;
#endif

	gpio_set_direction(_clkpin.port, _clkpin.pinnum, GPIO_FUNCTION);
	gpio_set_function(_clkpin.port, _clkpin.pinnum, function);

	if(_conf->controller != SPI_HALFDUPLEX_SLAVE){
		gpio_set_direction(_mosipin.port, _mosipin.pinnum, GPIO_FUNCTION);
		gpio_set_function(_mosipin.port, _mosipin.pinnum, function);
	}

	if(_conf->controller == SPI_HALFDUPLEX_MASTER){
		gpio_set_direction(_misopin.port, _misopin.pinnum, GPIO_FUNCTION);
		gpio_set_function(_misopin.port, _misopin.pinnum, function);
	}

	if(_conf->nss == SPI_HARDWARE_NSS){
		gpio_set_direction(_nsspin.port, _nsspin.pinnum, GPIO_FUNCTION);
		gpio_set_function(_nsspin.port, _nsspin.pinnum, function);
	}
#endif

	return ret;
}

void spi::hardware_deinitialize(void){
#if defined(SPI1)
	if     (_instance == SPI1) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
#endif /* defined(SPI1) */
#if defined(SPI2)
	else if(_instance == SPI2) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
#endif /* defined(SPI2) */
#if defined(SPI3)
	else if(_instance == SPI3) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);
#endif /* defined(SPI3) */
#if defined(SPI4)
	else if(_instance == SPI4) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI4EN);
#endif /* defined(SPI4) */
#if defined(SPI5)
	else if(_instance == SPI5) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI5EN);
#endif /* defined(SPI6) */
#if defined(SPI4)
	else if(_instance == SPI6) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI6EN);
#endif /* defined(SPI4) */

	gpio_deinit(_clkpin.port, _clkpin.pinnum);
	if(_conf->controller & (SPI_FULLDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE | SPI_HALFDUPLEX_SLAVE))
		gpio_deinit(_misopin.port, _misopin.pinnum);
	if(_conf->controller & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE))
		gpio_deinit(_mosipin.port, _mosipin.pinnum);
	if(_conf->nss == SPI_HARDWARE_NSS)
		gpio_deinit(_nsspin.port, _nsspin.pinnum);
}

err_t spi::initialize(spi_config_t *conf){
	err_t ret;

	_conf = conf;
	ret = hardware_initialize();
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
	CLEAR_REG(_instance->CR1);
	SET_BIT(_instance->CR1, (_conf->clocksample << SPI_CR1_CPHA_Pos) | (_conf->clockdivision << SPI_CR1_BR_Pos));
	SET_BIT(_instance->CR1, (_conf->datasize << SPI_CR1_DFF_Pos) | (_conf->bitorder << SPI_CR1_LSBFIRST_Pos));
	if(_conf->controller & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER)) SET_BIT(_instance->CR1, SPI_CR1_MSTR | SPI_CR1_SSI);
	if(_conf->controller & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))  SET_BIT(_instance->CR1, SPI_CR1_BIDIMODE);
	if(_conf->nss == SPI_SOFTWARE_NSS) SET_BIT(_instance->CR1, SPI_CR1_SSM);

	CLEAR_REG(_instance->CR2);
	if((_conf->controller & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER)) && (_conf->nss == SPI_HARDWARE_NSS)) SET_BIT(_instance->CR1, SPI_CR2_SSOE);

	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_FAIL);
#if CONFIG_PERIPH_SPI_LOG
		LOG_MESS(LOG_ERROR, TAG, "Interrupt priority out of range");
#endif /* CONFIG_PERIPH_SPI_LOG */
		system_reset();
		return ret;
	}
	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	_txmutex = xSemaphoreCreateMutex();
	_rxmutex = xSemaphoreCreateMutex();

	return ret;
}

void spi::deinitilize(void){
    hardware_deinitialize();

#if PERIPHERAL_DMA_AVAILABLE
	_txdma = NULL;
	_rxdma = NULL;
#endif /* PERIPHERAL_DMA_AVAILABLE */

	CLEAR_REG(_instance->CR1);
	CLEAR_REG(_instance->CR2);
	__IO uint32_t tmp = READ_REG(_instance->DR);
	tmp = READ_REG(_instance->SR);
	(void)tmp;

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);

    xSemaphoreTake(_txmutex, portMAX_DELAY);
    xSemaphoreTake(_rxmutex, portMAX_DELAY);
    vSemaphoreDelete(_txmutex);
    vSemaphoreDelete(_rxmutex);
}




void spi::register_event_handler(std::function<void(spi_event_t event, void *param)> event_handler_function, void *param){
	_event_handler = event_handler_function;
	_event_parameter = param;
}

void spi::unregister_event_handler(void){
	_event_handler = NULL;
	_event_parameter = NULL;
}

void spi::event_handle(spi_event_t event){
	if(_event_handler) _event_handler(event, _event_parameter);
}





#if PERIPHERAL_DMA_AVAILABLE
void spi::install_dma(dma_t txdma, dma_config_t *txdma_conf, dma_t rxdma, dma_config_t *rxdma_conf){
	_txdma = txdma;
	_rxdma = rxdma;
	_txdma_conf = txdma_conf;
	_rxdma_conf = rxdma_conf;

	if(_txdma_conf != NULL){
		_txdma_conf->direction = DMA_MEM_TO_PERIPH;
		_txdma_conf->datasize  = (dma_datasize_t)_conf->datasize;
		_txdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_txdma->initialize(_txdma_conf);
		_txdma->register_event_handler(FUNC_BIND(&spi::txdma_event_handler, this, FUNC_PARAM(1), FUNC_PARAM(2)), NULL);
	}
	if(_rxdma_conf != NULL){
		_rxdma_conf->direction = DMA_PERIPH_TO_MEM;
		_rxdma_conf->datasize  = (dma_datasize_t)_conf->datasize;
		_rxdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_rxdma->initialize(_rxdma_conf);
		_rxdma->register_event_handler(FUNC_BIND(&spi::rxdma_event_handler, this, FUNC_PARAM(1), FUNC_PARAM(2)), NULL);
	}
}

void spi::uninstall_dma(void){

	if(_txdma_conf != NULL){
		_txdma->deinitialize();
		_txdma->unregister_event_handler();
	}
	if(_rxdma_conf != NULL){
		_rxdma->deinitialize();
		_rxdma->unregister_event_handler();
	}
	_txdma = NULL;
	_rxdma = NULL;

	_txdma_conf = NULL;
	_rxdma_conf = NULL;
}

#endif /* PERIPHERAL_DMA_AVAILABLE */




SPI_TypeDef *spi::get_instance(void){
	return _instance;
}

spi_config_t *spi::get_config(void){
	return _conf;
}

IRQn_Type spi::get_irq(void){
	return _IRQn;
}
#if PERIPHERAL_DMA_AVAILABLE
dma_t spi::get_txdma(void){
	return _txdma;
}

dma_t spi::get_rxdma(void){
	return _rxdma;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */
SemaphoreHandle_t spi::get_txmutex(void){
	return _txmutex;
}

SemaphoreHandle_t spi::get_rxmutex(void){
	return _rxmutex;
}





err_t spi::transmit(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	spi_event_t event = SPI_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == (SPI_HALFDUPLEX_MASTER))
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		SET_BIT(_instance->CR1, SPI_CR1_SPE);

		while(txinfo.count < txinfo.length){
			ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_TXE, BITS_SET, timeout);
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				SPI_DBG("SPI wait flag timeout");
				event = SPI_EVENT_ERROR;
				goto EventCB;
			}

			_instance->DR = *(uint32_t *)(txinfo.buffer);
			txinfo.count++;
			if(_conf->datasize == SPI_DATASIZE_8BIT) 	   txinfo.buffer += sizeof(uint8_t);
			else if(_conf->datasize == SPI_DATASIZE_16BIT) txinfo.buffer += sizeof(uint16_t);
		}

		ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_BSY, BITS_RESET, timeout);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI wait flag timeout");
			event = SPI_EVENT_ERROR;
			goto EventCB;
		}

		if(!(_conf->controller & SPI_HALFDUPLEX_MASTER)){
			__IO uint32_t tmp = READ_REG(_instance->DR);
			tmp = READ_REG(_instance->SR);
			(void)tmp;
		}
		event = SPI_EVENT_TRANSMIT_COMPLETE;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);

	return ret;
}

err_t spi::receive(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	spi_event_t event = SPI_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master mode can't receive data");


	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == (SPI_HALFDUPLEX_MASTER))
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		SET_BIT(_instance->CR1, SPI_CR1_SPE);

		while(rxinfo.count < rxinfo.length){
			if(_conf->controller & SPI_FULLDUPLEX_MASTER){
				ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_TXE, BITS_SET, timeout);
				IS_ERROR(ret) {
					ERROR_CAPTURE(ret);
					SPI_DBG("SPI wait flag timeout");
					event = SPI_EVENT_ERROR;
					goto EventCB;
				}
				_instance->DR = 0x00UL;
			}

			ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_RXNE, BITS_SET, timeout);
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				SPI_DBG("SPI wait flag timeout");
				event = SPI_EVENT_ERROR;
				goto EventCB;
			}
			*(uint32_t *)rxinfo.buffer = _instance->DR;
			rxinfo.count++;
			if(_conf->datasize == SPI_DATASIZE_8BIT) rxinfo.buffer += sizeof(uint8_t);
			else if(_conf->datasize == SPI_DATASIZE_16BIT) rxinfo.buffer += sizeof(uint16_t);
		}

		ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_BSY, BITS_RESET, timeout);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI wait flag timeout");
			event = SPI_EVENT_ERROR;
			goto EventCB;
		}

		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		event = SPI_EVENT_RECEIVE_COMPLETE;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);

	return ret;
}

err_t spi::transmit_receive(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	spi_event_t event = SPI_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER || _conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master/slave mode can't transmit and receive data");


	(in_it == pdTRUE)?
			(err = (xSemaphoreTakeFromISR(_txmutex, NULL) && xSemaphoreTakeFromISR(_rxmutex, NULL))) :
			(err = (xSemaphoreTake(_txmutex, timeout) && xSemaphoreTake(_rxmutex, timeout)));
	if(err == pdTRUE){
		txinfo.buffer = ptxdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		rxinfo.buffer = prxdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		SET_BIT(_instance->CR1, SPI_CR1_SPE);
		while(txinfo.count < txinfo.length && rxinfo.count < rxinfo.length){
			ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_TXE, BITS_SET, timeout);
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				SPI_DBG("SPI wait flag timeout");
				event = SPI_EVENT_ERROR;
				goto EventCB;
			}
			_instance->DR = *(uint32_t *)txinfo.buffer;

			ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_RXNE, BITS_SET, timeout);
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				SPI_DBG("SPI wait flag timeout");
				event = SPI_EVENT_ERROR;
				goto EventCB;
			}
			*(uint32_t *)rxinfo.buffer = _instance->DR;

			if(_conf->datasize == SPI_DATASIZE_8BIT) {
				txinfo.buffer += sizeof(uint8_t);
				rxinfo.buffer += sizeof(uint8_t);
			}
			else if(_conf->datasize == SPI_DATASIZE_16BIT) {
				txinfo.buffer += sizeof(uint16_t);
				rxinfo.buffer += sizeof(uint16_t);
			}
			txinfo.count++;
			rxinfo.count++;
		}

		ret = wait_bits_fromISR(&(_instance->SR), SPI_SR_BSY, BITS_RESET, timeout);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI wait flag timeout");
			event = SPI_EVENT_ERROR;
			goto EventCB;
		}

		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		event = (spi_event_t)(SPI_EVENT_TRANSMIT_COMPLETE | SPI_EVENT_RECEIVE_COMPLETE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);

	return ret;
}




err_t spi::transmit_it(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == SPI_HALFDUPLEX_MASTER)
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		SET_BIT(_instance->CR2, SPI_CR2_TXEIE | SPI_CR2_ERRIE);

		__NVIC_ClearPendingIRQ(_IRQn);
		__NVIC_EnableIRQ(_IRQn);

		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}

err_t spi::receive_it(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master mode can't receive data");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == SPI_HALFDUPLEX_MASTER)
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		SET_BIT(_instance->CR2, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

		__NVIC_ClearPendingIRQ(_IRQn);
		__NVIC_EnableIRQ(_IRQn);

		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}

err_t spi::transmit_receive_it(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER || _conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master/slave mode can't transmit and receive data");


	(in_it == pdTRUE)?
			(err = (xSemaphoreTakeFromISR(_txmutex, NULL) && xSemaphoreTakeFromISR(_rxmutex, NULL))) :
			(err = (xSemaphoreTake(_txmutex, timeout) && xSemaphoreTake(_rxmutex, timeout)));
	if(err == pdTRUE){
		txinfo.buffer = ptxdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		rxinfo.buffer = prxdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		SET_BIT(_instance->CR2, SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE);

		__NVIC_EnableIRQ(_IRQn);
		__NVIC_ClearPendingIRQ(_IRQn);

		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}




#if PERIPHERAL_DMA_AVAILABLE
err_t spi::transmit_dma(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");
	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == SPI_HALFDUPLEX_MASTER)
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		CLEAR_BIT(_instance->CR2, SPI_CR2_TXDMAEN);

		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)txinfo.buffer,
			.pdest = (uint32_t)&_instance->DR,
			.xfersize = txinfo.length,
		};
		ret = _txdma->config_start_session(xfer_conf);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI DMA error");
			_txdma->stop_session();
			event_handle(SPI_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
			return ret;
		}

		SET_BIT(_instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_ERRIE);
		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}

err_t spi::receive_dma(uint32_t pdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");
	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(_conf->controller == SPI_HALFDUPLEX_MASTER)
			SET_BIT(_instance->CR1, SPI_CR1_BIDIOE);
		CLEAR_BIT(_instance->CR2, SPI_CR2_RXDMAEN);
		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)&_instance->DR,
			.pdest = (uint32_t)rxinfo.buffer,
			.xfersize = rxinfo.length,
		};
		ret = _rxdma->config_start_session(xfer_conf);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI DMA error");
			_rxdma->stop_session();
			event_handle(SPI_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
			return ret;
		}

		SET_BIT(_instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_ERRIE);
		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}

err_t spi::transmit_receive_dma(uint32_t ptxdata, uint32_t prxdata, uint32_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER || _conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master/slave mode can't transmit and receive data");
	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL || _rxdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	(in_it == pdTRUE)?
			(err = (xSemaphoreTakeFromISR(_txmutex, NULL) && xSemaphoreTakeFromISR(_rxmutex, NULL))) :
			(err = (xSemaphoreTake(_txmutex, timeout) && xSemaphoreTake(_rxmutex, timeout)));
	if(err == pdTRUE){
		txinfo.buffer = ptxdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		rxinfo.buffer = prxdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		CLEAR_BIT(_instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

		struct dma_session_config_t rxfer_conf = {
			.psource = (uint32_t)&_instance->DR,
			.pdest = (uint32_t)rxinfo.buffer,
			.xfersize = rxinfo.length,
		};
		ret = _rxdma->config_start_session(rxfer_conf);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI DMA error");
			_rxdma->stop_session();
			event_handle(SPI_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
			return ret;
		}

		struct dma_session_config_t txfer_conf = {
			.psource = (uint32_t)rxinfo.buffer,
			.pdest = (uint32_t)&_instance->DR,
			.xfersize = rxinfo.length,
		};
		ret = _txdma->config_start_session(txfer_conf);
		IS_ERROR(ret) {
			ERROR_CAPTURE(ret);
			SPI_DBG("SPI DMA error");
			_txdma->stop_session();
			event_handle(SPI_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
			return ret;
		}

		SET_BIT(_instance->CR2, SPI_CR2_ERRIE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
		SET_BIT(_instance->CR1, SPI_CR1_SPE);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		SPI_DBG("SPI operation was blocked by mutex");
	}

	return ret;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */




#if PERIPHERAL_DMA_AVAILABLE
err_t spi::txdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	if(READ_BIT(_instance->CR2, SPI_CR2_TXDMAEN)) {
		CLEAR_BIT(_instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);

		_txdma->stop_session();
	}
	return ret;
}

err_t spi::rxdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	if(READ_BIT(_instance->CR2, SPI_CR2_RXDMAEN)) {
		CLEAR_BIT(_instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);

		_rxdma->stop_session();
	}

	return ret;
}

void spi::txdma_event_handler(dma_event_t event, void *param){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		volatile uint32_t tmp = _instance->SR;
		tmp = _instance->DR;
		(void)tmp;

		dma_config_t *dma_conf = _txdma->get_config();
		if(dma_conf->mode != DMA_MODE_CIRCULAR) txdma_stop();
		event_handle(SPI_EVENT_TRANSMIT_COMPLETE);
		if(dma_conf->mode != DMA_MODE_CIRCULAR) xSemaphoreGiveFromISR(_txmutex, NULL);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		txdma_stop();
		event_handle(SPI_EVENT_ERROR);
		xSemaphoreGiveFromISR(_txmutex, NULL);
	}
}

void spi::rxdma_event_handler(dma_event_t event, void *param){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		volatile uint32_t tmp = _instance->SR;
		tmp = _instance->DR;
		(void)tmp;

		dma_config_t *dma_conf = _rxdma->get_config();
		rxinfo.count = rxinfo.length;

		if(dma_conf->mode != DMA_MODE_CIRCULAR) rxdma_stop();
		event_handle(SPI_EVENT_RECEIVE_COMPLETE);
		if(dma_conf->mode != DMA_MODE_CIRCULAR) xSemaphoreGiveFromISR(_rxmutex, NULL);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		rxdma_stop();
		event_handle(SPI_EVENT_ERROR);
		xSemaphoreGiveFromISR(_rxmutex, NULL);
	}

}
#endif /* PERIPHERAL_DMA_AVAILABLE */

err_t spi::abort_transmit_it(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, 1));
	if(err != pdTRUE && READ_BIT(_instance->CR2, SPI_CR2_TXEIE)){
		txinfo.count = 0U;
		txinfo.length = 0U;
		txinfo.buffer = 0U;

		CLEAR_BIT(_instance->CR2, SPI_CR2_TXEIE | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(!READ_BIT(_instance->CR2, SPI_CR2_RXNEIE)){
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_DisableIRQ(_IRQn);
		}

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
	}

	return ret;
}

err_t spi::abort_receive_it(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI mode halfduplex master mode can't receive data");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, 1));
	if(err != pdTRUE && READ_BIT(_instance->CR2, SPI_CR2_RXNEIE)){
		txinfo.count = 0U;
		txinfo.length = 0U;
		txinfo.buffer = 0U;

		CLEAR_BIT(_instance->CR2, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);
		if(!READ_BIT(_instance->CR2, SPI_CR2_TXEIE)){
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_DisableIRQ(_IRQn);
		}

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
	}

	return ret;
}

err_t spi::abort_all_it(void){
	err_t ret;

	ret = abort_transmit_it();
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		SPI_DBG("SPI abort error");
		return ret;
	}
	ret = abort_receive_it();
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		SPI_DBG("SPI abort error");
		return ret;
	}

	return ret;
}

#if PERIPHERAL_DMA_AVAILABLE
err_t spi::abort_transmit_dma(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_SLAVE),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");
	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, 1));
	if((err != pdTRUE) && READ_BIT(_instance->CR2, SPI_CR2_TXDMAEN)){
		txinfo.count = 0U;
		txinfo.length = 0;
		txinfo.buffer = 0;

		CLEAR_BIT(_instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);

		ret = txdma_stop();

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
	}

	return ret;
}

err_t spi::abort_receive_dma(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_conf->controller == SPI_HALFDUPLEX_MASTER),
				ERROR_SET(ret, E_NOT_ALLOWED), ret, LOG_ERROR, TAG, "SPI halfduplex slave mode can't transmit data");
	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "SPI DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, 1));
	if((err != pdTRUE) && READ_BIT(_instance->CR2, SPI_CR2_RXDMAEN)){
		rxinfo.count = 0U;
		rxinfo.length = 0;
		rxinfo.buffer = 0;

		CLEAR_BIT(_instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_ERRIE);
		CLEAR_BIT(_instance->CR1, SPI_CR1_SPE);

		ret = rxdma_stop();

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
	}

	return ret;
}

err_t spi::abort_all_dma(void){
	err_t ret;

	ret = abort_transmit_dma();
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		SPI_DBG("SPI abort error");
		return ret;
	}
	ret = abort_receive_dma();
	IS_ERROR(ret) {
		ERROR_CAPTURE(ret);
		SPI_DBG("SPI abort error");
		return ret;
	}

	return ret;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */



static void SPI_TXE_IRQHandler(spi_t pspi){
	err_t ret;
	spi_event_t event = SPI_EVENT_NOEVENT;
	SPI_TypeDef *spi = pspi->get_instance();
	spi_config_t *conf = pspi->get_config();
	__IO uint32_t cr2_reg = spi->CR2;
	__IO uint32_t sr_reg  = spi->SR;

/** SPI transmit event */
	if(READ_BIT(cr2_reg, SPI_CR2_TXEIE) && READ_BIT(sr_reg, SPI_SR_TXE)){
		if(conf->controller != SPI_HALFDUPLEX_SLAVE){
			if(pspi->rxinfo.count < pspi->rxinfo.length){
				spi->DR = *(__IO uint32_t *)(pspi->txinfo.buffer);
				pspi->txinfo.count++;

				if(pspi->txinfo.count >= pspi->txinfo.length){
					ret = wait_bits_fromISR(&(spi->SR), SPI_SR_TXE, BITS_SET, SPI_TIMEOUT);
					IS_ERROR(ret){
						ERROR_CAPTURE(ret);
						SPI_DBG("SPI wait flag timeout");
						event = SPI_EVENT_ERROR;
						goto spi_event_callback;
					}

					ret = wait_bits_fromISR(&(spi->SR), SPI_SR_BSY, BITS_RESET, SPI_TIMEOUT);
					IS_ERROR(ret){
						ERROR_CAPTURE(ret);
						SPI_DBG("SPI wait flag timeout");
						event = SPI_EVENT_ERROR;
						goto spi_event_callback;
					}

					if(conf->controller != SPI_HALFDUPLEX_MASTER){
						__IO uint32_t tmp = spi->DR;
						tmp = spi->SR;
						(void)tmp;
					}

					event = SPI_EVENT_TRANSMIT_COMPLETE;
					goto spi_event_callback;
				}

				if(conf->datasize ==  SPI_DATASIZE_8BIT)
					pspi->txinfo.buffer += sizeof(uint8_t);
				else if(conf->datasize ==  SPI_DATASIZE_16BIT)
					pspi->txinfo.buffer += sizeof(uint16_t);
				return;
			}
			return;
		}
		return;
	}
	return;

	spi_event_callback:
	CLEAR_BIT(spi->CR2, SPI_CR2_TXEIE);

	if(!READ_BIT(spi->CR2, SPI_CR2_RXNEIE)){
		__NVIC_ClearPendingIRQ(pspi->get_irq());
		__NVIC_DisableIRQ(pspi->get_irq());
		CLEAR_BIT(spi->CR2, SPI_CR2_ERRIE);
		CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
	}

	pspi->event_handle(event);

	pspi->txinfo.count = 0U;
	pspi->txinfo.length = 0U;
	pspi->txinfo.buffer = 0U;

	xSemaphoreGiveFromISR(pspi->get_txmutex(), NULL);

	return;
}

static void SPI_RXNE_IRQHandler(spi_t pspi){
	err_t ret;
	spi_event_t event = SPI_EVENT_NOEVENT;
	SPI_TypeDef *spi = pspi->get_instance();
	spi_config_t *conf = pspi->get_config();
	__IO uint32_t cr2_reg = spi->CR2;
	__IO uint32_t sr_reg  = spi->SR;

/** SPI receive event */
	if(READ_BIT(cr2_reg, SPI_CR2_RXNEIE) && READ_BIT(sr_reg, SPI_SR_RXNE)){
		if(conf->controller != SPI_HALFDUPLEX_MASTER){
			if(pspi->rxinfo.count < pspi->rxinfo.length){
				*(uint32_t *)(pspi->rxinfo.buffer) = spi->DR;
				pspi->rxinfo.count++;

				if(pspi->rxinfo.count >= pspi->rxinfo.length){
					ret = wait_bits_fromISR(&(spi->SR), SPI_SR_RXNE, BITS_RESET, SPI_TIMEOUT);
					IS_ERROR(ret){
						ERROR_CAPTURE(ret);
						SPI_DBG("SPI wait flag timeout");
						event = SPI_EVENT_ERROR;
						goto spi_event_callback;
					}

					__IO uint32_t tmp = spi->DR;
					tmp = spi->SR;
					(void)tmp;

					event = SPI_EVENT_RECEIVE_COMPLETE;
					goto spi_event_callback;
				}

				if(conf->datasize ==  SPI_DATASIZE_8BIT)
					pspi->rxinfo.buffer += sizeof(uint8_t);
				else if(conf->datasize ==  SPI_DATASIZE_16BIT)
					pspi->rxinfo.buffer += sizeof(uint16_t);
				return;
			}
			return;
		}
		return;
	}
	return;

spi_event_callback:
	CLEAR_BIT(spi->CR2, SPI_CR2_RXNEIE);

	if(!READ_BIT(spi->CR2, SPI_CR2_TXEIE)){
		__NVIC_ClearPendingIRQ(pspi->get_irq());
		__NVIC_DisableIRQ(pspi->get_irq());
		CLEAR_BIT(spi->CR2, SPI_CR2_ERRIE);
		CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
	}

	pspi->event_handle(event);

	pspi->rxinfo.count = 0U;
	pspi->rxinfo.length = 0U;
	pspi->rxinfo.buffer = 0U;

	xSemaphoreGiveFromISR(pspi->get_rxmutex(), NULL);
}

static void SPI_ERR_IRQHandler(spi_t pspi){
	SPI_TypeDef *spi = pspi->get_instance();
	__IO uint32_t sr_reg  = spi->SR;

/** SPI error overrun event */
	if(READ_BIT(sr_reg, SPI_SR_OVR)){
		SPI_DBG("SPI Overrun error, stopped all operating.");
		goto error_handle;
	}
	if(READ_BIT(sr_reg, SPI_SR_UDR)){
		SPI_DBG("SPI Underrun error, stopped all operating.");
		goto error_handle;
	}
	if(READ_BIT(sr_reg, SPI_SR_MODF)){
		SPI_DBG("I2C Mode fault, stopped all operating.");
		goto error_handle;
	}
#if defined(STM32F4)
	if(READ_BIT(sr_reg, SPI_SR_FRE)){
		SPI_DBG("I2C Frame format error, stopped all operating.");
		goto error_handle;
	}
#endif /* STM32F4 */

error_handle:
	__IO uint32_t tmp = READ_REG(spi->DR);
	tmp = READ_REG(spi->SR);
	(void)tmp;

	__NVIC_ClearPendingIRQ(pspi->get_irq());
	__NVIC_DisableIRQ(pspi->get_irq());
	CLEAR_BIT(spi->CR2, SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE);
	CLEAR_BIT(spi->CR1, SPI_CR1_SPE);

	pspi->event_handle(SPI_EVENT_ERROR);

	pspi->txinfo.count = 0U;
	pspi->txinfo.length = 0U;
	pspi->txinfo.buffer = 0U;
	pspi->rxinfo.count = 0U;
	pspi->rxinfo.length = 0U;
	pspi->rxinfo.buffer = 0U;

	xSemaphoreGiveFromISR(pspi->get_txmutex(), NULL);
	xSemaphoreGiveFromISR(pspi->get_rxmutex(), NULL);
}




static void SPICommon_IRQHandler(spi_t pspi){
	SPI_TXE_IRQHandler(pspi);
	SPI_RXNE_IRQHandler(pspi);
	SPI_ERR_IRQHandler(pspi);
}


#if PERIPHERAL_DMA_AVAILABLE

#endif /* PERIPHERAL_DMA_AVAILABLE */



#if defined(SPI1)
static spi spi_1(SPI1);
spi_t spi1 = &spi_1;
void SPI1_IRQHandler(void){
	SPICommon_IRQHandler(spi1);
}
#endif /* defined(SPI1) */
#if defined(SPI2)
static spi spi_2(SPI2);
spi_t spi2 = &spi_2;
void SPI2_IRQHandler(void){
	SPICommon_IRQHandler(spi2);
}
#endif /* defined(SPI2) */
#if defined(SPI3)
static spi spi_3(SPI3);
spi_t spi3 = &spi_3;
void SPI3_IRQHandler(void){
	SPICommon_IRQHandler(spi3);
}
#endif /* defined(SPI3) */
#if defined(SPI4)
static spi spi_4(SPI4);
spi_t spi4 = &spi_4;
void SPI4_IRQHandler(void){
	SPICommon_IRQHandler(spi4);
}
#endif /* defined(SPI4) */
#if defined(SPI5)
static spi spi_5(SPI5);
spi_t spi5 = &spi_5;
void SPI5_IRQHandler(void){
	SPICommon_IRQHandler(spi5);
}
#endif /* defined(SPI5) */
#if defined(SPI6)
static spi spi_6(SPI6);
spi_t spi6 = &spi_6;
void SPI6_IRQHandler(void){
	SPICommon_IRQHandler(spi6);
}
#endif /* defined(SPI6) */


#endif /* PERIPHERAL_SPI_AVAILABLE */

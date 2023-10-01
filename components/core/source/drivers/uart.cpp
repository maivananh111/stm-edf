/*
 * uart.cpp
 *
 *  Created on: 20 thg 11, 2022
 *      Author: anh
 */
#include "drivers/uart.h"
#if PERIPHERAL_UART_AVAILABLE

#include "math.h"
#include "string.h"

#include "drivers/system.h"
#include "drivers/systick.h"
#include "drivers/rcc.h"
#include "common/bits_check.h"
#include "common/expression_check.h"
#include "common/macro.h"
#if CONFIG_PERIPH_UART_LOG
#include "common/log_monitor.h"
#endif /* CONFIG_PERIPH_UART_LOG */


#define USART_TIMEOUT 100U

static const char *TAG = "UART";


static void UART_TXE_IRQHandler(uart_t puart);
static void UART_TXC_IRQHandler(uart_t puart);
static void UART_RXNE_IRQHandler(uart_t puart);
static void UART_IDLE_IRQHandler(uart_t puart);
static void UART_ERR_IRQHandler(uart_t puart);
static void UARTCommon_IRQHandler(uart_t puart);


#ifdef CONFIG_PERIPH_UART_LOG
#define UART_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, TAG, __MSG__);
#else
#define UART_DBG(__MSG__)\
{}
#endif


uart_t uart_str_decode(char *str){
#if defined(USART1)
	if(strcasecmp(str, "UART1") == 0)
		return uart1;
#endif /* defined(USART1) */
#if defined(USART2)
	if(strcasecmp(str, "UART2") == 0)
		return uart2;
#endif /* defined(USART2) */
#if defined(USART3)
	if(strcasecmp(str, "UART3") == 0)
		return uart3;
#endif /* defined(USART3) */
#if defined(UART4)
	if(strcasecmp(str, "UART4") == 0)
		return uart4;
#endif /* defined(UART4) */
#if defined(UART5)
	if(strcasecmp(str, "UART5") == 0)
		return uart5;
#endif /* defined(UART5) */
#if defined(USART6)
	if(strcasecmp(str, "UART6") == 0)
		return uart6;
#endif /* defined(USART6) */
#if defined(UART7)
	if(strcasecmp(str, "UART7") == 0)
		return uart7;
#endif /* defined(UART7) */
#if defined(UART8)
	if(strcasecmp(str, "UART8") == 0)
		return uart8;
#endif /* defined(UART8) */

	return uart1;
}

uart::uart(USART_TypeDef *uart){
	_instance = uart;
}

err_t uart::hardware_initialize(void){
	err_t ret;

	if(gpio_str_decode(_conf->txpin, &_txpin) == false ||
	   gpio_str_decode(_conf->rxpin, &_rxpin) == false){
		ERROR_SET(ret, E_INVALID);
		UART_DBG("UART pin invalid");
		system_reset();
		return ret;
	}
	gpio_port_clock_enable(_txpin.port);
	gpio_port_clock_enable(_rxpin.port);

#if defined(STM32F1)
	gpio_set_direction(_txpin.port, _txpin.pinnum, GPIO_FUNCTION);
	gpio_set_direction(_txpin.port, _txpin.pinnum, GPIO_INPUT);
#elif defined(STM32F4)
	if(
#if defined(USART1)
			_instance == USART1
#endif /* defined(USART1) */
			||
#if defined(USART2)
			_instance == USART2
#endif /* defined(USART2) */
			||
#if defined(USART3)
			_instance == USART3
#endif /* defined(USART3) */
	){
		gpio_set_direction(_txpin.port, _txpin.pinnum, GPIO_FUNCTION);
		gpio_set_function(_txpin.port, _txpin.pinnum, AF7_UART1_3);
		gpio_set_direction(_rxpin.port, _rxpin.pinnum, GPIO_FUNCTION);
		gpio_set_function(_rxpin.port, _rxpin.pinnum, AF7_UART1_3);
	}
	else{
		gpio_set_direction(_txpin.port, _txpin.pinnum, GPIO_FUNCTION);
		gpio_set_direction(_rxpin.port, _rxpin.pinnum, GPIO_FUNCTION);
#if defined(UART7) & defined(UART8)
		gpio_set_function(_rxpin.port, _rxpin.pinnum, AF8_UART4_8);
		gpio_set_function(_txpin.port, _txpin.pinnum, AF8_UART4_8);
#else
		gpio_set_function(_rxpin.port, _rxpin.pinnum, AF8_UART4_6);
		gpio_set_function(_txpin.port, _txpin.pinnum, AF8_UART4_6);
#endif /* defined(UART7) & defined(UART8) */
	}
#endif /* STM32F4 */

#if defined(USART1)
	if(_instance == USART1){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB2);
		_IRQn = USART1_IRQn;
	}
#endif /* defined(USART1) */
#if defined(USART2)
	if(_instance == USART2){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = USART2_IRQn;
	}
#endif /* defined(USART2) */
#if defined(USART3)
	if(_instance == USART3){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = USART3_IRQn;
	}
#endif /* defined(USART3) */
#if defined(UART4)
	if(_instance == UART4){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = UART4_IRQn;
	}
#endif /* defined(UART4) */
#if defined(UART5)
	if(_instance == UART5){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = UART5_IRQn;
	}
#endif /* defined(UART5) */
#if defined(USART6)
	if(_instance == USART6){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB2);
		_IRQn = USART6_IRQn;
	}
#endif /* defined(USART6) */
#if defined(UART7)
	if(_instance == UART7){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART7EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = UART7_IRQn;
	}
#endif /* defined(UART7) */
#if defined(UART8)
	if(_instance == UART8){
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART8EN);
		_bus_freq_hz = rcc_get_bus_frequency(BUS_APB1);
		_IRQn = UART8_IRQn;
	}
#endif /* defined(UART8) */

	return ret;
}

void uart::hardware_deinitialize(void){
#if defined(USART1)
	if(_instance == USART1) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
#endif /* defined(USART1) */
#if defined(USART2)
	if(_instance == USART2) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
#endif /* defined(USART2) */
#if defined(USART3)
	if(_instance == USART3) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
#endif /* defined(USART3) */
#if defined(UART4)
	if(_instance == UART4) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);
#endif /* defined(UART4) */
#if defined(UART5)
	if(_instance == UART5) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);
#endif /* defined(UART5) */
#if defined(USART6)
	if(_instance == USART6) CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
#endif /* defined(USART6) */
#if defined(UART7)
	if(_instance == UART7) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART7EN);
#endif /* defined(UART7) */
#if defined(UART8)
	if(_instance == UART8) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART8EN);
#endif /* defined(UART8) */

	gpio_deinit(_txpin.port, _txpin.pinnum);
	gpio_deinit(_rxpin.port, _rxpin.pinnum);
}

err_t uart::initialize(uart_config_t *conf){
	err_t ret;

	_conf = conf;
	ret = hardware_initialize();
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
	CLEAR_REG(_instance->CR1);
	SET_BIT(_instance->CR1, USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

	float USARTDIV = (float)(_bus_freq_hz/(_conf->baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	WRITE_REG(_instance->BRR, (DIV_Mantissa << 4) | (DIV_Fraction << 0));

	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
		UART_DBG("Interrupt priority out of range");
		system_reset();
		return ret;
	}

	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	_txmutex = xSemaphoreCreateMutex();
	_rxmutex = xSemaphoreCreateMutex();

	return {};
}

err_t uart::deinitialize(void){
#if PERIPHERAL_DMA_AVAILABLE
	_txdma = NULL;
	_rxdma = NULL;
#endif /* PERIPHERAL_DMA_AVAILABLE */

	CLEAR_REG(_instance->CR1);
	CLEAR_REG(_instance->CR2);
	CLEAR_REG(_instance->BRR);
	__IO uint32_t tmp = READ_REG(_instance->DR);
	tmp = READ_REG(_instance->SR);
	(void)tmp;

	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);

    xSemaphoreTake(_txmutex, portMAX_DELAY);
    xSemaphoreTake(_rxmutex, portMAX_DELAY);
    vSemaphoreDelete(_txmutex);
    vSemaphoreDelete(_rxmutex);

	return {};
}



void uart::register_event_handler(std::function<void(uart_event_t event, void *param)> event_handler_function, void *param){
	_event_handler = event_handler_function;
	_event_parameter = param;
}

void uart::unregister_event_handler(void){
	_event_handler = NULL;
	_event_parameter = NULL;
}

void uart::event_handle(uart_event_t event){
	if(_event_handler) _event_handler(event, _event_parameter);
}



#if PERIPHERAL_DMA_AVAILABLE
void uart::install_dma(dma_t txdma, dma_config_t *txdma_conf, dma_t rxdma, dma_config_t *rxdma_conf){
	_txdma = txdma;
	_rxdma = rxdma;

	_txdma_conf = txdma_conf;
	_rxdma_conf = rxdma_conf;
	if(_txdma_conf != NULL){
		_txdma_conf->direction = DMA_MEM_TO_PERIPH;
		_txdma_conf->datasize  = DMA_DATASIZE_8BIT;
		_txdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_txdma->initialize(_txdma_conf);
		_txdma->register_event_handler(FUNC_BIND(&uart::txdma_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
	if(_rxdma_conf != NULL){
		_rxdma_conf->direction = DMA_PERIPH_TO_MEM;
		_rxdma_conf->datasize  = DMA_DATASIZE_8BIT;
		_rxdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_rxdma->initialize(_rxdma_conf);
		_rxdma->register_event_handler(FUNC_BIND(&uart::rxdma_event_handler, this, std::placeholders::_1, std::placeholders::_2), NULL);
	}
}

void uart::uninstall_dma(void){
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



USART_TypeDef *uart::get_instance(void){
	return _instance;
}

uart_config_t *uart::get_config(void){
	return _conf;
}

IRQn_Type uart::get_irq(void){
	return _IRQn;
}

uart_reception_t uart::get_reception_mode(void){
	return _reception_mode;
}

char uart::get_endchar(void){
	return _endchar;
}
#if PERIPHERAL_DMA_AVAILABLE
dma_t uart::get_txdma(void){
	return _txdma;
}

dma_t uart::get_rxdma(void){
	return _rxdma;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */
SemaphoreHandle_t uart::get_txmutex(void){
	return _txmutex;
}

SemaphoreHandle_t uart::get_rxmutex(void){
	return _rxmutex;
}

err_t uart::get_rxbuffer(uint8_t **ppdata){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((ppdata == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "Assert error NULL parameter");

	if(rxinfo.buffer != NULL){
		rxinfo.buffer[rxinfo.count + 1] = '\0';
		*ppdata = (uint8_t *)malloc(rxinfo.count + 1);
		ASSERT_THEN_RETURN_VALUE((ppdata == NULL),
				ERROR_SET(ret, E_MEM), ret, LOG_ERROR, TAG, "Memory exhausted");
		memcpy(*ppdata, rxinfo.buffer, rxinfo.count+1);
		release_rxbuffer();
		return ret;
	}
	ERROR_SET(ret, E_NULL);
	UART_DBG("Buffer is NULL");

	return ret;
}

uint16_t uart::get_rxbuffersize(void){
	return rxinfo.count;
}



void uart::set_endchar(char endchar){
	_endchar = endchar;;
}

err_t uart::reload_rxbuffer(uint16_t new_buffer_size){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((new_buffer_size == 0),
			ERROR_SET(ret, E_INVALID), ret, LOG_ERROR, TAG, "Assert error invalid parameter");
	release_rxbuffer();
	rxinfo.length = new_buffer_size;
	rxinfo.buffer = (uint8_t *)malloc((rxinfo.length + 1) * sizeof(uint8_t));
	ASSERT_THEN_RETURN_VALUE((rxinfo.buffer == NULL),
			ERROR_SET(ret, E_MEM), ret, LOG_ERROR, TAG, "Memory exhausted");
	memset(rxinfo.buffer, '\0', rxinfo.length + 1);

	return ret;
}

void uart::release_rxbuffer(void){
	rxinfo.count = 0;
	if(rxinfo.buffer != NULL) {
		free(rxinfo.buffer);
		rxinfo.buffer = NULL;
	}
}




err_t uart::transmit(uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	uart_event_t event = UART_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		while(txinfo.count < txinfo.length) {
			_instance->DR = *txinfo.buffer++;

			ret = wait_bits_fromISR(&(_instance->SR), USART_SR_TC, BITS_SET, timeout);
			IS_ERROR(ret){
				UART_DBG("UART wait flag timeout");
				event = UART_EVENT_ERROR;
				goto EventCB;
			}
			txinfo.count++;
		}
		event = UART_EVENT_TRANSMIT_COMPLETE;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("UART operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);

	return ret;
}

err_t uart::receive(uart_reception_t reception, uint16_t data_size, uint16_t timeout){
	err_t ret;
	uart_event_t event = UART_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((reception == UART_RECEPTION_UNTIL_IDLE),
			ERROR_SET(ret, E_NOT_SUPPORTED), ret, LOG_ERROR, TAG, "UART reception until idle not supported in normal mode");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		reload_rxbuffer(data_size);
		_reception_mode = reception;
		while(rxinfo.count < rxinfo.length){
			ret = wait_bits_fromISR(&(_instance->SR), USART_SR_RXNE, BITS_SET, timeout);
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				release_rxbuffer();
				event = UART_EVENT_ERROR;
				goto EventCB;
			}
			uint8_t dt = _instance->DR;
			if(_reception_mode == UART_RECEPTION_UNTIL_ENDCHAR){
				if(dt == _endchar){
					event = UART_EVENT_ENDCHAR;
					goto EventCB;
				}
			}
			(*rxinfo.buffer++) = dt;
			rxinfo.count++;
		}
		event = UART_EVENT_RECEIVE_COMPLETE;
		goto EventCB;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("UART operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);

	return ret;
}

err_t uart::transmit_it(uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		SET_BIT(_instance->CR1, USART_CR1_TCIE | USART_CR1_TXEIE);
		SET_BIT(_instance->CR3, USART_CR3_EIE);

		__NVIC_ClearPendingIRQ(_IRQn);
		__NVIC_EnableIRQ(_IRQn);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("DMA session was blocked by mutex");
	}

	return ret;
}

err_t uart::receive_it(uart_reception_t reception, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		reload_rxbuffer(data_size);
		_reception_mode = reception;

		SET_BIT(_instance->CR1, USART_CR1_RXNEIE);
		if(_reception_mode == UART_RECEPTION_UNTIL_IDLE)
			SET_BIT(_instance->CR1, USART_CR1_IDLEIE);
		SET_BIT(_instance->CR3, USART_CR3_EIE);

		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		__NVIC_ClearPendingIRQ(_IRQn);
		__NVIC_EnableIRQ(_IRQn);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("UART operation was blocked by mutex");
	}

	return ret;
}

#if PERIPHERAL_DMA_AVAILABLE
err_t uart::transmit_dma(uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAT);
		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)txinfo.buffer,
			.pdest = (uint32_t)&_instance->DR,
			.xfersize = txinfo.length,
		};
		ret = _txdma->config_start_session(xfer_conf);
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			UART_DBG("UART DMA error");
			_txdma->stop_session();
			event_handle(UART_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
			return ret;
		}
		CLEAR_BIT(_instance->SR, USART_SR_TC);
		SET_BIT(_instance->CR3, USART_CR3_DMAT);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("UART operation was blocked by mutex");
	}
	return ret;
}

err_t uart::receive_dma(uart_reception_t reception, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");
	ASSERT_THEN_RETURN_VALUE((reception == UART_RECEPTION_UNTIL_ENDCHAR),
			ERROR_SET(ret, E_NOT_SUPPORTED), ret, LOG_ERROR, TAG, "UART reception until end char not supported in dma mode");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, timeout));
	if(err == pdTRUE){
		reload_rxbuffer(data_size);
		_reception_mode = reception;

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAR);
		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)&_instance->DR,
			.pdest = (uint32_t)rxinfo.buffer,
			.xfersize = rxinfo.length,
		};
		ret = _rxdma->config_start_session(xfer_conf);
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			UART_DBG("UART DMA error");
			release_rxbuffer();
			_rxdma->stop_session();
			event_handle(UART_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
			return ret;
		}
		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		SET_BIT(_instance->CR3, USART_CR3_DMAR);
		if(_reception_mode == UART_RECEPTION_UNTIL_IDLE) {
			SET_BIT(_instance->CR1, USART_CR1_IDLEIE);
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_EnableIRQ(_IRQn);
		}
	}
	else{
		ERROR_SET(ret, E_BUSY);
		UART_DBG("UART operation was blocked by mutex");
	}

	return ret;
}


err_t uart::txdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");

	if(READ_BIT(_instance->CR3, USART_CR3_DMAT)){
		_txdma->stop_session();

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAT);
	}

	return ret;
}

err_t uart::rxdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");

	if(READ_BIT(_instance->CR3, USART_CR3_DMAR)){
		_rxdma->stop_session();

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAR);
	}

	return ret;
}

void uart::txdma_event_handler(dma_event_t event, void *param){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		dma_config_t *dma_conf = _txdma->get_config();
		if(dma_conf->mode != DMA_MODE_CIRCULAR) txdma_stop();
		event_handle(UART_EVENT_TRANSMIT_COMPLETE);
		if(dma_conf->mode != DMA_MODE_CIRCULAR) xSemaphoreGiveFromISR(_txmutex, NULL);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		txdma_stop();
		event_handle(UART_EVENT_ERROR);
		xSemaphoreGiveFromISR(_txmutex, NULL);
	}
}

void uart::rxdma_event_handler(dma_event_t event, void *param){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		dma_config_t *dma_conf = _rxdma->get_config();
		rxinfo.count = rxinfo.length;

		if(dma_conf->mode != DMA_MODE_CIRCULAR) rxdma_stop();
		event_handle(UART_EVENT_RECEIVE_COMPLETE);
		if(dma_conf->mode == DMA_MODE_CIRCULAR) reload_rxbuffer();
		if(dma_conf->mode != DMA_MODE_CIRCULAR) xSemaphoreGiveFromISR(_rxmutex, NULL);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		if(_instance->CR1 & USART_CR1_IDLEIE) _instance->CR1 &=~ USART_CR1_IDLEIE;
		rxdma_stop();
		event_handle(UART_EVENT_ERROR);
		release_rxbuffer();
		xSemaphoreGiveFromISR(_rxmutex, NULL);
	}
}
#endif /* PERIPHERAL_DMA_AVAILABLE */



void uart::abort_transmit_it(void){
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, 1));
	if(err != pdTRUE && READ_BIT(_instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE))){
		txinfo.buffer = NULL;
		txinfo.length = 0;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR1, USART_CR1_TXEIE | USART_CR1_TCIE);
		if(!READ_BIT(_instance->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE)){
			CLEAR_BIT(_instance->CR3, USART_CR3_EIE);
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_DisableIRQ(_IRQn);
		}

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
	}
}

void uart::abort_receive_it(void){
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, 1));
	if(err != pdTRUE && (_instance->CR1 & (USART_CR1_RXNEIE | USART_CR1_IDLEIE))){
		release_rxbuffer();

		CLEAR_BIT(_instance->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE);
		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		if(!READ_BIT(_instance->CR1, USART_CR1_TXEIE | USART_CR1_TCIE)){
			CLEAR_BIT(_instance->CR3, USART_CR3_EIE);
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_DisableIRQ(_IRQn);
		}

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
	}
}

void uart::abort_all_it(void){
	abort_transmit_it();
	abort_receive_it();
}

err_t uart::abort_transmit_dma(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_txmutex, NULL)) : (err = xSemaphoreTake(_txmutex, 1));
	if((err != pdTRUE) && READ_BIT(_instance->CR3, USART_CR3_DMAT)){
		txinfo.buffer = NULL;
		txinfo.length = 0;
		txinfo.count = 0U;

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAT);
		CLEAR_BIT(_instance->SR, USART_SR_TC);

		_txdma->stop_session();

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_txmutex, NULL) : xSemaphoreGive(_txmutex);
	}

	return ret;
}

err_t uart::abort_receive_dma(void){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "UART DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_rxmutex, NULL)) : (err = xSemaphoreTake(_rxmutex, 1));
	if((err != pdTRUE) && READ_BIT(_instance->CR3, USART_CR3_DMAR)){
		release_rxbuffer();

		CLEAR_BIT(_instance->CR3, USART_CR3_DMAR);
		__IO uint32_t tmp = READ_REG(_instance->DR);
		tmp = READ_REG(_instance->SR);
		(void)tmp;

		_rxdma->stop_session();

		if(_reception_mode == UART_RECEPTION_UNTIL_IDLE) {
			CLEAR_BIT(_instance->CR1, USART_CR1_IDLEIE);
			if(!READ_BIT(_instance->CR1, USART_CR1_TXEIE | USART_CR1_TCIE)){
				__NVIC_ClearPendingIRQ(_IRQn);
				__NVIC_DisableIRQ(_IRQn);
			}
		}
		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_rxmutex, NULL) : xSemaphoreGive(_rxmutex);
	}

	return ret;
}

err_t uart::abort_all_dma(void){
	err_t ret;

	ret = abort_transmit_dma();
	IS_ERROR(ret)
		UART_DBG("Fail to abort UART transmit DMA");
	ret = abort_receive_dma();
	IS_ERROR(ret)
		UART_DBG("Fail to abort UART receive DMA");


	return ret;
}




static void UART_TXE_IRQHandler(uart_t puart){
	USART_TypeDef *uart = puart->get_instance();
	uint32_t SRReg  = READ_REG(uart->SR);
	uint32_t CR1Reg = READ_REG(uart->CR1);

	if(READ_BIT(SRReg, USART_SR_TXE) && READ_BIT(CR1Reg, USART_CR1_TXEIE)){
		CLEAR_BIT(uart->SR, USART_SR_TXE);
		if(puart->txinfo.count < puart->txinfo.length){
			uart->DR = puart->txinfo.buffer[puart->txinfo.count];
			puart->txinfo.count++;
		}
	}
}

static void UART_TXC_IRQHandler(uart_t puart){
	USART_TypeDef *uart = puart->get_instance();
	uint32_t SRReg  = uart->SR;
	uint32_t CR1Reg = uart->CR1;

	if(READ_BIT(SRReg, USART_SR_TC) && READ_BIT(CR1Reg, USART_CR1_TCIE)) {
		CLEAR_BIT(uart->SR, USART_SR_TC);
		CLEAR_BIT(uart->CR1,USART_CR1_TXEIE | USART_CR1_TCIE);
		__IO uint32_t tmp = uart->DR;
		(void)tmp;
		if(!READ_BIT(uart->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE)){
			CLEAR_BIT(uart->CR3, USART_CR3_EIE);
			__NVIC_ClearPendingIRQ(puart->get_irq());
			__NVIC_DisableIRQ(puart->get_irq());
		}
		puart->event_handle(UART_EVENT_TRANSMIT_COMPLETE);
		xSemaphoreGiveFromISR(puart->get_txmutex(), NULL);
	}
}

static void UART_RXNE_IRQHandler(uart_t puart){
	USART_TypeDef *uart = puart->get_instance();
	uint32_t SRReg  = READ_REG(uart->SR);
	uint32_t CR1Reg = READ_REG(uart->CR1);

	if(READ_BIT(SRReg, USART_SR_RXNE) && READ_BIT(CR1Reg, USART_CR1_RXNEIE)) {
		CLEAR_BIT(uart->SR, USART_SR_RXNE);
		if(puart->rxinfo.count < puart->rxinfo.length){
			puart->rxinfo.buffer[puart->rxinfo.count] = uart->DR;
			if(puart->rxinfo.count == puart->rxinfo.length){
				__IO uint32_t tmp = READ_REG(uart->DR);
				(void)tmp;
				CLEAR_BIT(uart->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE);
				if(!READ_BIT(CR1Reg, USART_CR1_TXEIE | USART_CR1_TCIE)){
					CLEAR_BIT(uart->CR3, USART_CR3_EIE);
					__NVIC_ClearPendingIRQ(puart->get_irq());
					__NVIC_DisableIRQ(puart->get_irq());
				}
				puart->event_handle(UART_EVENT_RECEIVE_COMPLETE);
				xSemaphoreGiveFromISR(puart->get_rxmutex(), NULL);
				puart->release_rxbuffer();
				return;
			}
			if(((puart->get_reception_mode() == UART_RECEPTION_UNTIL_ENDCHAR) && (puart->rxinfo.buffer[puart->rxinfo.count] == puart->get_endchar()))){
				__IO uint32_t tmp = READ_REG(uart->DR);
				(void)tmp;
				CLEAR_BIT(uart->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE);
				if(!READ_BIT(CR1Reg, USART_CR1_TXEIE | USART_CR1_TCIE)){
					CLEAR_BIT(uart->CR3, USART_CR3_EIE);
					__NVIC_ClearPendingIRQ(puart->get_irq());
					__NVIC_DisableIRQ(puart->get_irq());
				}
				puart->event_handle(UART_EVENT_ENDCHAR);
				xSemaphoreGiveFromISR(puart->get_rxmutex(), NULL);
				puart->release_rxbuffer();
				return;
			}
			puart->rxinfo.count++;
			return;
		}
		return;
	}
}

static void UART_IDLE_IRQHandler(uart_t puart){
	USART_TypeDef *uart = puart->get_instance();
	uint32_t SRReg  = READ_REG(uart->SR);
	uint32_t CR1Reg = READ_REG(uart->CR1);
	uint32_t CR3Reg = READ_REG(uart->CR3);

	if(READ_BIT(SRReg, USART_SR_IDLE) && READ_BIT(CR1Reg, USART_CR1_IDLEIE)) {
		__IO uint32_t tmp = READ_REG(uart->DR);
		(void)tmp;
		uart->SR &=~ (USART_SR_IDLE | USART_SR_RXNE);
		uart->CR1 &=~ (USART_CR1_IDLEIE | USART_CR1_RXNEIE);
#if PERIPHERAL_DMA_AVAILABLE
		if(CR3Reg & USART_CR3_DMAR){
			dma_t rxdma = puart->get_rxdma();
			puart->rxinfo.count = puart->rxinfo.length - rxdma->get_transfer_counter();
			if(rxdma->get_config()->mode != DMA_MODE_CIRCULAR) puart->rxdma_stop();
		}
#endif /* PERIPHERAL_DMA_AVAILABLE */
		if(!(CR1Reg & (USART_CR1_TXEIE | USART_CR1_TCIE))){
			uart->CR3 &=~ USART_CR3_EIE;
			__NVIC_ClearPendingIRQ(puart->get_irq());
			__NVIC_DisableIRQ(puart->get_irq());
		}
		puart->event_handle(UART_EVENT_IDLE);
		xSemaphoreGiveFromISR(puart->get_rxmutex(), NULL);
		puart->release_rxbuffer();
	}
}

static void UART_ERR_IRQHandler(uart_t puart){
	USART_TypeDef *uart = puart->get_instance();
	uint32_t SRReg  = uart->SR;

	if(SRReg & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)){
		uart->CR1 &=~ (USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_TXEIE);
		uart->CR3 &=~ USART_CR3_EIE;

		__NVIC_ClearPendingIRQ(puart->get_irq());
		__NVIC_DisableIRQ(puart->get_irq());

		puart->event_handle(UART_EVENT_ERROR);
		xSemaphoreGiveFromISR(puart->get_txmutex(), NULL);
		xSemaphoreGiveFromISR(puart->get_rxmutex(), NULL);
		puart->release_rxbuffer();
	}
}

static void UARTCommon_IRQHandler(uart_t puart){
	UART_TXE_IRQHandler(puart);
	UART_TXC_IRQHandler(puart);
	UART_RXNE_IRQHandler(puart);
	UART_IDLE_IRQHandler(puart);
	UART_ERR_IRQHandler(puart);
}






#if defined(USART1)
static uart uart_1(USART1);
uart_t uart1 = &uart_1;
void USART1_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_1);
}
#endif /* defined(USART1) */
#if defined(USART2)
static uart uart_2(USART2);
uart_t uart2 = &uart_2;
void USART2_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_2);
}
#endif /* defined(USART2) */
#if defined(USART3)
static uart uart_3(USART3);
uart_t uart3 = &uart_3;
void USART3_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_3);
}
#endif /* defined(USART3) */
#if defined(UART4)
static uart uart_4 (UART4);
uart_t uart4 = &uart_4;
void UART4_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_4);
}
#endif /* defined(UART4) */
#if defined(UART5)
static uart uart_5 (UART5);
uart_t uart5 = &uart_5;
void UART5_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_5);
}
#endif /* defined(UART5) */
#if defined(USART6)
static uart uart_6(USART6);
uart_t uart6 = &uart_6;
void USART6_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_6);
}
#endif /* defined(USART6) */
#if defined(UART7)
static uart uart_7 (UART7);
uart_t uart7 = &uart_7;
void UART7_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_7);
}
#endif /* defined(UART7) */
#if defined(UART8)
static uart uart_8 (UART8);
uart_t uart8 = &uart_8;
void UART8_IRQHandler(void){
	UARTCommon_IRQHandler(&uart_8);
}
#endif /* defined(UART8) */

#endif /* ENABLE_USART */






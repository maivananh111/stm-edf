/*
 * i2c.cpp
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */
#include "drivers/i2c.h"
#if PERIPHERAL_I2C_AVAILABLE

#include "string.h"

#include "drivers/rcc.h"
#include "drivers/system.h"
#include "common/bits_check.h"
#include "common/expression_check.h"
#include "common/macro.h"
#if CONFIG_PERIPH_I2C_LOG
#include "common/log_monitor.h"
#endif /* CONFIG_PERIPH_I2C_LOG */


static const char *TAG = "I2C";

#define I2C_BUSY_TIMEOUT      1000U
#define I2C_TIMEOUT           1000U
#define I2C_MIN_FREQ_STANDARD 2000000U // Minimum is 2MHz of APB1 bus in standard mode.
#define I2C_MIN_FREQ_FAST     4000000U // Minimum is 4MHz of APB1 bus in standard mode.


#ifdef CONFIG_PERIPH_I2C_LOG
#define I2C_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, TAG, __MSG__);
#else
#define I2C_DBG(__MSG__)\
{}
#endif


static void I2C_Master_TX_IRQHandler(i2c_t pi2c);
static void I2C_Master_RX_IRQHandler(i2c_t pi2c);

static void I2C_Slave_TX_IRQHandler(i2c_t pi2c);
static void I2C_Slave_RX_IRQHandler(i2c_t pi2c);

static void I2CCommonEvent_IRQHandler(i2c_t pi2c);
static void I2CCommonError_IRQHandler(i2c_t pi2c);

#if PERIPHERAL_DMA_AVAILABLE
static void i2c_txdma_event_handler(dma_event_t event, void *param);
static void i2c_rxdma_event_handler(dma_event_t event, void *param);
#endif /* PERIPHERAL_DMA_AVAILABLE */



i2c_t i2c_str_decode(char *str){
#if defined(I2C1)
	if(strcasecmp(str, "I2C1") == 0)
		return i2c1;
#endif /* defined(I2C1) */
#if defined(I2C2)
	if(strcasecmp(str, "I2C2") == 0)
		return i2c2;
#endif /* defined(I2C2) */
#if defined(I2C3)
	if(strcasecmp(str, "I2C3") == 0)
		return i2c3;
#endif /* defined(I2C3) */
#if defined(I2C4)
	if(strcasecmp(str, "I2C4") == 0)
		return i2c4;
#endif /* defined(I2C4) */
#if defined(I2C5)
	if(strcasecmp(str, "I2C5") == 0)
		return i2c5;
#endif /* defined(I2C5) */
#if defined(I2C6)
	if(strcasecmp(str, "I2C6") == 0)
		return i2c6;
#endif /* defined(I2C6) */

	return i2c1;
}

i2c::i2c(I2C_TypeDef *i2c){
	_instance = i2c;
}

err_t i2c::initialize(i2c_config_t *conf){
	err_t ret;

	_conf = conf;
#if defined(I2C1)
	if(_instance == I2C1)      SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
#endif /* defined(I2C1) */
#if defined(I2C2)
	else if(_instance == I2C2) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
#endif /* defined(I2C2) */
#if defined(I2C3)
	else if(_instance == I2C3) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);
#endif /* defined(I2C3) */

	if(_conf->sclpin != NULL)
		if(gpio_str_decode(_conf->sclpin, &_sclpin) == false) goto error;
	if(_conf->sdapin != NULL)
		if(gpio_str_decode(_conf->sdapin, &_sdapin) == false) goto error;
	goto pass;
	error:
	ERROR_SET(ret, E_INVALID);
	I2C_DBG("I2C pin invalid");
	system_reset();
	return ret;

	pass:
	gpio_set_direction(_sclpin.port, _sclpin.pinnum, GPIO_FUNCTION);
	gpio_set_direction(_sdapin.port, _sdapin.pinnum, GPIO_FUNCTION);
	gpio_set_type(_sclpin.port, _sclpin.pinnum, GPIO_OUTPUT_OPENDRAIN);
	gpio_set_type(_sdapin.port, _sdapin.pinnum, GPIO_OUTPUT_OPENDRAIN);
#if defined(STM32F4)
	gpio_set_function(_sclpin.port, _sclpin.pinnum, AF4_I2C1_3);
	gpio_set_function(_sdapin.port, _sdapin.pinnum, AF4_I2C1_3);
#endif
	gpio_set_pullup(_sclpin.port, _sclpin.pinnum);
	gpio_set_pullup(_sdapin.port, _sdapin.pinnum);

	CLEAR_REG(_instance->CR1);
	CLEAR_BIT(_instance->CR1, I2C_CR1_PE);
	SET_BIT(_instance->CR1, I2C_CR1_SWRST);
	delay_ms(5);
	CLEAR_BIT(_instance->CR1, I2C_CR1_SWRST);

	uint32_t apb1_freq = rcc_get_bus_frequency(BUS_APB1);
	if(((_conf->speedmode == I2C_STANDARD_MODE)? (apb1_freq < I2C_MIN_FREQ_STANDARD) : (apb1_freq < I2C_MIN_FREQ_FAST))) {
		ERROR_SET(ret, E_INVALID);
		I2C_DBG("I2C Frequency invalid");
		return ret;
	}
	CLEAR_REG(_instance->CR2);
	WRITE_REG(_instance->CR2, (uint32_t)((apb1_freq / 1000000U) >> I2C_CR2_FREQ_Pos));
	WRITE_REG(_instance->TRISE, (uint32_t)((_conf-> speedmode== I2C_STANDARD_MODE)? ((apb1_freq/1000000U) + 1U) : ((((apb1_freq/1000000U) * 300U) / 1000U) + 1U)));
	CLEAR_REG(_instance->CCR);
	CLEAR_BIT(_instance->CCR, I2C_CCR_DUTY);

	__IO uint32_t _ccr_reg = 0UL;
	if(_conf-> speedmode == I2C_STANDARD_MODE){
		CLEAR_BIT(_instance->CCR, I2C_CCR_FS);
		// T_high = T_low = CCR * T_PCLK1, T = 2 * T_high.
		_ccr_reg = (uint32_t)((apb1_freq - 1U)/(2U * _conf->frequency) + 1U);
		if(_ccr_reg < 4U) _ccr_reg = 4U;
	}
	else if(_conf-> speedmode == I2C_FAST_MODE){
		SET_BIT(_instance->CCR, I2C_CCR_FS);
		// T_high = (1/2)T_low = CCR * TPCLK1, T = 3 * T_high.
		_ccr_reg = (uint32_t)((apb1_freq)/(3U * _conf->frequency));
		if(_ccr_reg < 1U) _ccr_reg = 1U;
	}
	SET_BIT(_instance->CCR, _ccr_reg);

	(_conf->generalcall)? SET_BIT(_instance->CR1, I2C_CR1_ENGC)
						: CLEAR_BIT(_instance->CR1, I2C_CR1_ENGC);
	CLEAR_BIT(_instance->CR1, I2C_CR1_NOSTRETCH);
	CLEAR_BIT(_instance->OAR1, I2C_OAR1_ADDMODE);
	if(_conf->slavemode_address != 0U) SET_BIT(_instance->OAR1, (_conf->slavemode_address << I2C_OAR1_ADD1_Pos));
#if defined(STM32F4) && defined(STM32F429xx)
	(_conf->analogfilter == true)? CLEAR_BIT(_instance->FLTR, I2C_FLTR_ANOFF) : SET_BIT(_instance->FLTR, I2C_FLTR_ANOFF);
	WRITE_REG(_instance->FLTR, (uint32_t)((_instance->FLTR & ~I2C_FLTR_DNF)| _conf->digitalnoisefilter));
#endif
	#if defined(I2C1)
	if(_instance == I2C1){
		_evIRQn = I2C1_EV_IRQn;
		_erIRQn = I2C1_ER_IRQn;
	}
#endif /* defined(I2C1) */
#if defined(I2C2)
	else if(_instance == I2C2){
		_evIRQn = I2C2_EV_IRQn;
		_erIRQn = I2C2_ER_IRQn;
	}
#endif /* defined(I2C2) */
#if defined(I2C3)
	else if(_instance == I2C3){
		_evIRQn = I2C3_EV_IRQn;
		_erIRQn = I2C3_ER_IRQn;
	}
#endif /* defined(I2C3) */
	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
		I2C_DBG("Interrupt priority out of range");
		system_reset();
		return ret;
	}
	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_evIRQn, _conf->interruptpriority);
	__NVIC_SetPriority(_erIRQn, _conf->interruptpriority);

	SET_BIT(_instance->CR1, I2C_CR1_PE);
	CLEAR_REG(_instance->SR1);
	CLEAR_REG(_instance->SR2);

	_mutex = xSemaphoreCreateMutex();

	return ret;
}

void i2c::deinitialize(void){
#if defined(I2C1)
	if(_instance == I2C1)      CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
#endif /* defined(I2C1) */
#if defined(I2C2)
	else if(_instance == I2C2) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
#endif /* defined(I2C2) */
#if defined(I2C3)
	else if(_instance == I2C3) CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);
#endif /* defined(I2C3) */

#if PERIPHERAL_DMA_AVAILABLE
	_txdma = NULL;
	_rxdma = NULL;
#endif /* PERIPHERAL_DMA_AVAILABLE */
	gpio_deinit(_sclpin.port, _sclpin.pinnum);
	gpio_deinit(_sdapin.port, _sdapin.pinnum);

	CLEAR_REG(_instance->CR1);
	CLEAR_REG(_instance->CR2);
	CLEAR_REG(_instance->OAR1);
	CLEAR_REG(_instance->OAR2);
	CLEAR_REG(_instance->CCR);
	CLEAR_REG(_instance->TRISE);
#if defined(STM32F4) && defined(STM32F429xx)
	CLEAR_REG(_instance->FLTR);
#endif

	__NVIC_ClearPendingIRQ(_evIRQn);
	__NVIC_ClearPendingIRQ(_erIRQn);
	__NVIC_DisableIRQ(_evIRQn);
	__NVIC_DisableIRQ(_erIRQn);

    xSemaphoreTake(_mutex, portMAX_DELAY);
    vSemaphoreDelete(_mutex);
}




void i2c::register_event_handler(i2c_evcb_t event_handler_function, void *param){
	_event_handler = event_handler_function;
	_event_parameter = param;
}

void i2c::unregister_event_handler(void){
	_event_handler = NULL;
	_event_parameter = NULL;
}

void i2c::event_handle(i2c_event_t event){
	if(_event_handler) _event_handler(event, _event_parameter);
}




#if PERIPHERAL_DMA_AVAILABLE
void i2c::install_dma(dma_t txdma, dma_config_t *txdma_conf, dma_t rxdma, dma_config_t *rxdma_conf){
	_txdma = txdma;
	_rxdma = rxdma;

	_txdma_conf = txdma_conf;
	_rxdma_conf = rxdma_conf;
	if(_txdma_conf != NULL){
		_txdma_conf->direction = DMA_MEM_TO_PERIPH;
		_txdma_conf->datasize  = DMA_DATASIZE_8BIT;
		_txdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_txdma->initialize(_txdma_conf);
		_txdma->register_event_handler(i2c_txdma_event_handler, this);
	}
	if(_rxdma_conf != NULL){
		_rxdma_conf->direction = DMA_PERIPH_TO_MEM;
		_rxdma_conf->datasize  = DMA_DATASIZE_8BIT;
		_rxdma_conf->interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT;

		_rxdma->initialize(_rxdma_conf);
		_rxdma->register_event_handler(i2c_rxdma_event_handler, this);
	}
}

void i2c::uninstall_dma(void){
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



I2C_TypeDef *i2c::get_instance(void){
	return _instance;
}

i2c_config_t *i2c::get_config(void){
	return _conf;
}

IRQn_Type i2c::get_ev_irq(void){
	return _evIRQn;
}

IRQn_Type i2c::get_er_irq(void){
	return _erIRQn;
}

#if PERIPHERAL_DMA_AVAILABLE
dma_t i2c::get_txdma(void){
	return _txdma;
}

dma_t i2c::get_rxdma(void){
	return _rxdma;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */

SemaphoreHandle_t i2c::get_mutex(void){
	return _mutex;
}

i2c_action_t i2c::get_current_action(void){
	return _action;
}

uint8_t i2c::get_address(void){
	return _current_dev_address;
}



void i2c::ACK_failure_handle(void){
	I2C_DBG("I2C ACK error");
	SET_BIT(_instance->CR1, I2C_CR1_STOP);
	CLEAR_BIT(_instance->SR1, I2C_SR1_AF);
}

void i2c::clear_ADDR_flag(void){
	__IO uint32_t tmp = READ_REG(_instance->SR1 | _instance->SR2);
	(void)tmp;
}

err_t i2c::wait_for_ready(void){
	err_t ret;
	ERROR_SET(ret, E_BUSY);

	ret = wait_bits_fromISR(&(_instance->SR2), I2C_SR2_BUSY, BITS_RESET, I2C_BUSY_TIMEOUT);
	IS_PASS(ret)
		return ret;

	ERROR_SET(ret, E_BUSY);
	I2C_DBG("I2C Busy");

	return ret;
}

err_t i2c::wait_addr_flag(void){
	err_t ret;

	ret = checkbits_while_waitbits_fromISR(&(_instance->SR1), I2C_SR1_AF, BITS_SET, &(_instance->SR1), I2C_SR1_ADDR, BITS_SET, I2C_TIMEOUT);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		I2C_DBG("I2C address error");
		IS_E_FAIL(ret) ACK_failure_handle();
	}
	return ret;
}

err_t i2c::start(void){
	err_t ret;

	/** Master send start condition */
	SET_BIT(_instance->CR1, I2C_CR1_START);
	/** Wait for EV5 occur: SB=1 */
	ret = wait_bits_fromISR(&(_instance -> SR1), I2C_SR1_SB, BITS_SET, I2C_TIMEOUT);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		I2C_DBG("I2C error start");
	}

	return ret;
}

void i2c::stop(void){
	SET_BIT(_instance->CR1, I2C_CR1_STOP);
}

void i2c::send_address(uint8_t address, i2c_action_t action){
	/** Read SR1 register */
	__IO uint32_t tmpreg = READ_REG(_instance->SR1);
	(void)tmpreg;

	/** Write to DR register */
	(action == I2C_WRITE)? WRITE_REG(_instance->DR, (uint8_t)((address & 0x00FF) & ~I2C_OAR1_ADD0))
	                     : WRITE_REG(_instance->DR, (uint8_t)((address & 0x00FF) |  I2C_OAR1_ADD0));
}

void i2c::setup_reception(void){
	if(rxinfo.length == 1U){
		/** ACK=0 because last byte data come with NACK. */
		/** POS=0: ACK bit controls the (N)ACK of the current byte which will be received in the shift register. */
		CLEAR_BIT(_instance->CR1, I2C_CR1_ACK | I2C_CR1_POS);
	}
	else if(rxinfo.length == 2U){
		CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
		/** POS=1: ACK bit controls the (N)ACK of the next byte which will be received in the shift register. */
		SET_BIT(_instance->CR1, I2C_CR1_POS);
	}
	else{
		/** Enable ACK generate after each byte received. */
		SET_BIT(_instance->CR1, I2C_CR1_ACK);
		/** POS=0: ACK bit controls the (N)ACK of the current byte which will be received in the shift register. */
		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);
	}

	clear_ADDR_flag();
	/** In one byte receive mode, stop condition need to be generate after read byte. */
	if(rxinfo.length == 1U) stop();
}




err_t i2c::is_device_ready(uint8_t address, uint8_t num_trials, uint16_t timeout){
	err_t ret;
	uint8_t i = 1U;

	ret = wait_for_ready();
	IS_E_BUSY(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}
	/** POS=0: ACK bit controls the (N)ACK of the current byte which will be received in the shift register. */
	CLEAR_BIT(_instance->CR1, I2C_CR1_POS);

	do{
		ret = start();
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			return ret;
		}

		send_address(address, I2C_WRITE);

		ret = wait_addr_flag();
		IS_ERROR(ret) break;

		stop();
		if(READ_REG(_instance->SR1 & I2C_SR1_ADDR)){
			clear_ADDR_flag();
			ret = wait_for_ready();
			return ret;
		}
		else{
			ret = wait_for_ready();
			IS_ERROR(ret) return ret;
		}
		i++;
	} while(i < num_trials);

	ERROR_SET(ret, E_FAIL);

	return ret;
}

err_t i2c::transmit(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	i2c_event_t event = I2C_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0;
		_current_dev_address = address;
		_action = I2C_WRITE;

		/** POS=0: ACK bit controls the (N)ACK of the current byte which will be received in the shift register. */
		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);

		if(_conf->controller == I2C_MODE_MASTER){
			/** In master mode, master must be wait for I2C bus ready. */
			ret = wait_for_ready();
			IS_E_BUSY(ret){
				ERROR_CAPTURE(ret);
				return ret;
			}
			/** Start communication, Master send start condition. Wait for EV5 happen if start condition successful. */
			ret = start();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			/** Handle EV5: SB clear by read SR1 register, follow by write to DR register (Send slave address). */
			/** Master send slave address and decide enter transmitter or receiver mode by LSB bit address. TRA bit set by hardware if enter transmitter mode. */
			send_address(_current_dev_address, I2C_WRITE);
			/** Wait for EV6: ADDR set by hardware, clear by read SR1 and SR2 register. */
			ret = wait_addr_flag();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
		}
		else if(_conf->controller == I2C_MODE_SLAVE){
			/** Turn ACK response on to send ACK pulse if address matched. */
			SET_BIT(_instance->CR1, I2C_CR1_ACK);
			/** Wait ADDR set by address matched. */
			ret = wait_addr_flag();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
		}
		/** Read SR1 and SR2 register to clear ADDR bit, end EV6. */
		clear_ADDR_flag();
		/** Enter transmitter mode. */
		while(txinfo.count < txinfo.length){
			/** First byte need to be send after EV8_1: Wait for TxE set by ACK received after write data to DR register (Send slave address). It mean wait EV8_1. */
			/** During wait TxE set by hardware, if AF bit (NACK) is set then stop communication.  */
			ret = checkbits_while_waitbits(&(_instance->SR1), I2C_SR1_AF, BITS_SET, &(_instance->SR1), I2C_SR1_TXE, BITS_SET, I2C_TIMEOUT);
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				I2C_DBG("I2C error tx empty flag not set");
				IS_E_FAIL(ret) ACK_failure_handle();
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			/** Write data to DR register to send. */
			_instance->DR = *txinfo.buffer++;
			txinfo.count++;
		}

		if(_conf->controller == I2C_MODE_MASTER){
			/** Wait for last byte transmitted (BTF set by hardware), TxE is set and DR register is empty, SCL stretching. */
			ret = wait_bits_fromISR(&(_instance->SR1), I2C_SR1_BTF, BITS_SET, I2C_TIMEOUT);
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				I2C_DBG("I2C last byte transmit error");
				event = I2C_EVENT_ERROR;
				stop();
				goto EventCB;
			}
			/** End of communication. */
			stop();
		}
		else if(_conf->controller == I2C_MODE_SLAVE){
			/** EV8_2: At last by, master send NACK. Slave need to check NACK received (AF=1). */
			ret = wait_bits(&(_instance->SR1), I2C_SR1_AF, BITS_SET, I2C_TIMEOUT);
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				I2C_DBG("I2C ACK error");
				event = I2C_EVENT_ERROR;
				CLEAR_BIT(_instance->SR1, I2C_SR1_AF);
				CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
				goto EventCB;
			}
			CLEAR_BIT(_instance->SR1, I2C_SR1_AF);
			CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
		}

		event = I2C_EVENT_TRANSMIT_COMPLETE;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);

	return ret;
}

err_t i2c::receive(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	i2c_event_t event = I2C_EVENT_NOEVENT;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;
		_current_dev_address = address;
		_action = I2C_READ;

		/** POS=0: ACK bit controls the (N)ACK of the current byte which will be received in the shift register. */
		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);

		if(_conf->controller == I2C_MODE_MASTER){
			/** In master mode, master must be wait for I2C bus ready. */
			ret = wait_for_ready();
			IS_E_BUSY(ret){
				ERROR_CAPTURE(ret);
				return ret;
			}
			/** Start communication, Master send start condition. Wait for EV5 happen if start condition successful. */
			ret = start();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			/** Handle EV5: SB clear by read SR1 register, follow by write to DR register (Send slave address). */
			/** Master send slave address and decide enter transmitter or receiver mode by LSB bit address. TRA bit set by hardware if enter transmitter mode. */
			send_address(_current_dev_address, I2C_READ);
			/** Wait for EV6: ADDR set by hardware, clear by read SR1 and SR2 register. */
			ret = wait_addr_flag();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			/** Setup reception procedure. */
			setup_reception();
			/** Start read data process. */
			while(rxinfo.count < rxinfo.length){
				uint16_t remain = rxinfo.length - rxinfo.count;

				if(remain == 1U){
					/** Wait RxNE set byte hardware, check STOPF in slave mode. */
					ret = checkbits_while_waitbits(&(_instance->SR1), I2C_SR1_STOPF, BITS_SET, &(_instance->SR1), I2C_SR1_RXNE, BITS_SET, I2C_TIMEOUT);
					IS_ERROR(ret){
						IS_ERROR(ret) CLEAR_BIT(_instance->SR1, I2C_SR1_STOPF); /** Clear STOPF set by stop condition detected in slave mode. */
						ERROR_CAPTURE(ret);
						event = I2C_EVENT_ERROR;
						goto EventCB;
					}
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
				}
				else if(remain == 2U){
					/** Wait BTF set (Data byte N-1 in DR register has not been read and data byte N available in shift register. */
					ret = wait_bits(&(_instance->SR1), I2C_SR1_BTF, BITS_SET, I2C_TIMEOUT);
					IS_ERROR(ret) {
						ERROR_CAPTURE(ret);
						event = I2C_EVENT_ERROR;
						goto EventCB;
					}
					stop();
					/** Read DR register to get data byte N-1. */
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
					/** After read data byte N-1, data byte N available in DR register. */
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
				}
#if defined(STM32F4)
				else if(remain == 3U){
					/** Wait BTF set (Data byte N-2 in DR register has not been read and data byte N-1 available in shift register. */
					ret = wait_bits(&(_instance->SR1), I2C_SR1_BTF, BITS_SET, I2C_TIMEOUT);
					IS_ERROR(ret) {
						ERROR_CAPTURE(ret);
						event = I2C_EVENT_ERROR;
						goto EventCB;
					}
					/** Turn off ACK generate for 2 last bytes receive. */
					CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
					/** Read DR register to get data byte N-2. */
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
					/** Wait for BTF set by hardware (data N-1 in DR register and data N in shift register. */
					ret = wait_bits(&(_instance->SR1), I2C_SR1_BTF, BITS_SET, I2C_TIMEOUT);
					IS_ERROR(ret) {
						ERROR_CAPTURE(ret);
						event = I2C_EVENT_ERROR;
						goto EventCB;
					}
					stop();
					/** Read DR register to get data byte N-1. */
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
					/** Read DR register to get data byte N. */
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
				}
#endif /* STM32F4 */
				else{
					/** Wait RxNE set byte hardware, check STOPF in slave mode. */
					ret = checkbits_while_waitbits(&(_instance->SR1), I2C_SR1_STOPF, BITS_SET, &(_instance->SR1), I2C_SR1_RXNE, BITS_SET, I2C_TIMEOUT);
					IS_ERROR(ret){
						IS_ERROR(ret) CLEAR_BIT(_instance->SR1, I2C_SR1_STOPF); /** Clear STOPF set by stop condition detected in slave mode. */
						ERROR_CAPTURE(ret);
						event = I2C_EVENT_ERROR;
						goto EventCB;
					}
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
					if(get_bits(&(_instance->SR1), I2C_SR1_BTF) == I2C_SR1_BTF){
						*rxinfo.buffer++ = (uint8_t)_instance->DR;
						rxinfo.count++;
					}
				}
			}
		}
	/** Mode Slave ****************************************************************/
		else if(_conf->controller == I2C_MODE_SLAVE){
			_instance->CR1 |= I2C_CR1_ACK;

			ret = wait_addr_flag();
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			clear_ADDR_flag();

			while(rxinfo.count < rxinfo.length){
				ret = checkbits_while_waitbits(&(_instance->SR1), I2C_SR1_STOPF, BITS_SET, &(_instance->SR1), I2C_SR1_RXNE, BITS_SET, I2C_TIMEOUT);
				IS_ERROR(ret){
					ERROR_CAPTURE(ret);
					IS_E_FAIL(ret) CLEAR_BIT(_instance->SR1, I2C_SR1_STOPF);
					event = I2C_EVENT_ERROR;
					goto EventCB;
				}

				*rxinfo.buffer++ = (uint8_t)_instance->DR;
				rxinfo.count++;
				if(_instance->SR1 & I2C_SR1_BTF){
					*rxinfo.buffer++ = (uint8_t)_instance->DR;
					rxinfo.count++;
				}
			}

			ret = wait_bits(&(_instance->SR1), I2C_SR1_STOPF, BITS_SET, I2C_TIMEOUT);
			IS_ERROR(ret){
				ERROR_CAPTURE(ret);
				CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
				event = I2C_EVENT_ERROR;
				goto EventCB;
			}
			uint32_t tmpreg = READ_REG(_instance->SR1);
			(void)tmpreg;
			CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
		}

		event = I2C_EVENT_RECEIVE_COMPLETE;
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
		return ret;
	}

EventCB:
	event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);

	return ret;
}

err_t i2c::transmit_it(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;
		_current_dev_address = address;
		_action = I2C_WRITE;

		if(_conf->controller == I2C_MODE_MASTER){
			ret = wait_for_ready();
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				event_handle(I2C_EVENT_ERROR);
				(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
				return ret;
			}
		}

		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);
		__NVIC_ClearPendingIRQ(_evIRQn);
		__NVIC_ClearPendingIRQ(_erIRQn);
		__NVIC_EnableIRQ(_evIRQn);
		__NVIC_EnableIRQ(_erIRQn);
		if(_conf->controller == I2C_MODE_SLAVE) SET_BIT(_instance->CR1, I2C_CR1_ACK);
		SET_BIT(_instance->CR2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		if(_conf->controller == I2C_MODE_MASTER) SET_BIT(_instance->CR1, I2C_CR1_START);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
	}

	return ret;
}

err_t i2c::receive_it(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0;
		_current_dev_address = address;
		_action = I2C_READ;

		if(_conf->controller == I2C_MODE_MASTER){
			ret = wait_for_ready();
			IS_ERROR(ret) {
				ERROR_CAPTURE(ret);
				event_handle(I2C_EVENT_ERROR);
				(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
				return ret;
			}
		}

		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);
		__NVIC_ClearPendingIRQ(_evIRQn);
		__NVIC_ClearPendingIRQ(_erIRQn);
		__NVIC_EnableIRQ(_evIRQn);
		__NVIC_EnableIRQ(_erIRQn);
		SET_BIT(_instance->CR2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		SET_BIT(_instance->CR1, I2C_CR1_ACK);
		if(_conf->controller == I2C_MODE_MASTER) SET_BIT(_instance->CR1, I2C_CR1_START);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
	}

	return ret;
}

#if PERIPHERAL_DMA_AVAILABLE
err_t i2c::transmit_dma(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "I2C DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		txinfo.buffer = pdata;
		txinfo.length = data_size;
		txinfo.count = 0U;
		_current_dev_address = address;
		_action = I2C_WRITE;

		CLEAR_BIT(_instance->CR1, I2C_CR1_POS);
		ret = wait_for_ready();
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			event_handle(I2C_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
			return ret;
		}

		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)txinfo.buffer,
			.pdest = (uint32_t)&_instance->DR,
			.xfersize = txinfo.length,
		};
		ret = _txdma->config_start_session(xfer_conf);
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			I2C_DBG("I2C DMA error");
			event_handle(I2C_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
			return ret;
		}

		__NVIC_ClearPendingIRQ(_evIRQn);
		__NVIC_ClearPendingIRQ(_erIRQn);
		__NVIC_EnableIRQ(_evIRQn);
		__NVIC_EnableIRQ(_erIRQn);
		SET_BIT(_instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		SET_BIT(_instance->CR2, I2C_CR2_DMAEN);
		SET_BIT(_instance->CR1, I2C_CR1_ACK);
		if(_conf->controller == I2C_MODE_MASTER) SET_BIT(_instance->CR1, I2C_CR1_START);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
	}

	return ret;
}

err_t i2c::receive_dma(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
			ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "I2C DMA invalid");

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, timeout));
	if(err == pdTRUE){
		rxinfo.buffer = pdata;
		rxinfo.length = data_size;
		rxinfo.count = 0U;
		_current_dev_address = address;
		_action = I2C_READ;

		struct dma_session_config_t xfer_conf = {
			.psource = (uint32_t)&_instance->DR,
			.pdest = (uint32_t)rxinfo.buffer,
			.xfersize = rxinfo.length,
		};
		ret = _rxdma->config_start_session(xfer_conf);
		IS_ERROR(ret){
			ERROR_CAPTURE(ret);
			I2C_DBG("I2C DMA error");
			event_handle(I2C_EVENT_ERROR);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
			return ret;
		}

		__NVIC_ClearPendingIRQ(_evIRQn);
		__NVIC_ClearPendingIRQ(_erIRQn);
		__NVIC_EnableIRQ(_evIRQn);
		__NVIC_EnableIRQ(_erIRQn);
		SET_BIT(_instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		SET_BIT(_instance->CR2, I2C_CR2_DMAEN);
		SET_BIT(_instance->CR1, I2C_CR1_ACK);
		if(_conf->controller == I2C_MODE_MASTER) SET_BIT(_instance->CR1, I2C_CR1_START);
	}
	else{
		ERROR_SET(ret, E_BUSY);
		I2C_DBG("I2C operation was blocked by mutex");
	}

	return ret;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */





void i2c::abort_it(void){
	__NVIC_ClearPendingIRQ(_evIRQn);
	__NVIC_ClearPendingIRQ(_erIRQn);
	__NVIC_EnableIRQ(_evIRQn);
	__NVIC_EnableIRQ(_erIRQn);

	CLEAR_BIT(_instance->CR2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

	xSemaphoreGiveFromISR(_mutex, NULL);
}

void i2c::abort_dma(void){

}

#if PERIPHERAL_DMA_AVAILABLE
err_t i2c::txdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_txdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "I2C DMA invalid");

	if(READ_BIT(_instance->CR2, I2C_CR2_DMAEN)) {
		CLEAR_BIT(_instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		CLEAR_BIT(_instance->CR2, I2C_CR2_DMAEN);
		CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
		_txdma->stop_session();
	}

	return ret;
}

err_t i2c::rxdma_stop(void){
	err_t ret;

	ASSERT_THEN_RETURN_VALUE((_rxdma == NULL),
				ERROR_SET(ret, E_NULL), ret, LOG_ERROR, TAG, "I2C DMA invalid");

	if(READ_BIT(_instance->CR2, I2C_CR2_DMAEN)) {
		CLEAR_BIT(_instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
		CLEAR_BIT(_instance->CR2, I2C_CR2_DMAEN);
		CLEAR_BIT(_instance->CR1, I2C_CR1_ACK);
		_rxdma->stop_session();
	}

	return ret;
}
#endif /* PERIPHERAL_DMA_AVAILABLE */



static void I2C_Master_TX_IRQHandler(i2c_t pi2c){
	I2C_TypeDef *i2c = pi2c->get_instance();
	__IO uint32_t cr1reg = i2c->CR1;
	__IO uint32_t cr2reg = i2c->CR2;
	__IO uint32_t sr1reg = i2c->SR1;
	__IO uint32_t sr2reg = i2c->SR2;

	if(READ_BIT(sr2reg, I2C_SR2_TRA) && READ_BIT(cr1reg, I2C_CR2_ITBUFEN)
			&& READ_BIT(cr1reg, I2C_CR2_ITEVTEN) && !READ_BIT(cr2reg, I2C_CR2_DMAEN)){
		if((pi2c->txinfo.count < pi2c->txinfo.length) && READ_BIT(sr1reg, I2C_SR1_TXE)){
			i2c->DR = *pi2c->txinfo.buffer++;
			pi2c->txinfo.count++;
			if(pi2c->txinfo.count == pi2c->txinfo.length) goto EndOfTransmit;
		}
		if((pi2c->txinfo.count < pi2c->txinfo.length) && READ_BIT(sr1reg, I2C_SR1_BTF)){
			i2c->DR = *pi2c->txinfo.buffer++;
			pi2c->txinfo.count++;
			if(pi2c->txinfo.count == pi2c->txinfo.length) goto EndOfTransmit;
		}
		return;

EndOfTransmit:
		pi2c->stop();
		pi2c->event_handle(I2C_EVENT_TRANSMIT_COMPLETE);
		pi2c->abort_it();
	}
}

static void I2C_Master_RX_IRQHandler(i2c_t pi2c){
	err_t ret;
	I2C_TypeDef *i2c = pi2c->get_instance();
	__IO uint32_t cr1reg = i2c->CR1;
	__IO uint32_t cr2reg = i2c->CR2;
	__IO uint32_t sr1reg = i2c->SR1;
	__IO uint32_t sr2reg = i2c->SR2;

	if(!(sr2reg & I2C_SR2_TRA) && (cr1reg & I2C_CR2_ITBUFEN) && (cr1reg & I2C_CR2_ITEVTEN)
			 && !READ_BIT(cr2reg, I2C_CR2_DMAEN)&& (pi2c->rxinfo.length > pi2c->rxinfo.count)){
		uint16_t remain = pi2c->rxinfo.length - pi2c->rxinfo.count;

		if(sr1reg & I2C_SR1_RXNE){
			if(remain > 3U){
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;

				remain = pi2c->rxinfo.length - pi2c->rxinfo.count;
				if(remain == 3U) i2c->CR2 &=~ I2C_CR2_ITBUFEN;
			}
			else if(remain == 1U){
				i2c->CR1 &=~ I2C_CR1_ACK;
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;
				goto EndOfReceive;
			}
			else{
				i2c->CR2 &=~ I2C_CR2_ITBUFEN;
			}
		}

		else if(sr1reg & I2C_SR1_BTF){
			if(remain == 2U){
				i2c->CR2 &=~ I2C_CR2_ITBUFEN;
				pi2c->stop();
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;
				goto EndOfReceive;
			}
			else if(remain == 3U){
				i2c->CR2 &=~ I2C_CR2_ITBUFEN;
				i2c->CR1 &=~ I2C_CR1_ACK;
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;
			}
			else if(remain == 4U){
				i2c->CR2 &=~ I2C_CR2_ITBUFEN;
				*pi2c->rxinfo.buffer++ = (uint8_t)i2c->DR;
				pi2c->rxinfo.count++;
			}
		}
	}
EndOfReceive:
	pi2c->event_handle(I2C_EVENT_RECEIVE_COMPLETE);
	pi2c->abort_it();
}

static void I2C_Slave_TX_IRQHandler(i2c_t pi2c){

}

static void I2C_Slave_RX_IRQHandler(i2c_t pi2c){

}


static void I2CCommonEvent_IRQHandler(i2c_t pi2c){
	I2C_TypeDef *i2c = pi2c->get_instance();
	i2c_config_t *conf = pi2c-> get_config();
	__IO uint32_t sr1reg = i2c->SR1;
	uint8_t address = pi2c->get_address();
	i2c_action_t action = pi2c->get_current_action();

	if(conf->controller == I2C_MODE_MASTER){
		/** Start condition sent */
		if(sr1reg & I2C_SR1_SB){
			pi2c->send_address(address, action);
			return;
		}
		/** ADDR set */
		else if(sr1reg & I2C_SR1_ADDR){
			if(action == I2C_WRITE){
				__IO uint32_t tmp = i2c->SR1 | i2c->SR2;
				(void)tmp;
			}
			else if(action == I2C_READ) pi2c->setup_reception();
			return;
		}
		/** Transmitting */
		I2C_Master_TX_IRQHandler(pi2c);
		/** Receiving */
		I2C_Master_RX_IRQHandler(pi2c);
	}
	else if(conf->controller == I2C_MODE_SLAVE){
		/** Do Nothing */
		I2C_Slave_TX_IRQHandler(pi2c);
		I2C_Slave_RX_IRQHandler(pi2c);
	}
}

static void I2CCommonError_IRQHandler(i2c_t pi2c){
	I2C_TypeDef *i2c = pi2c->get_instance();
	i2c_config_t *conf = pi2c->get_config();
	__IO uint32_t sr1reg = i2c->SR1;

	if(sr1reg & I2C_SR1_BERR){
		I2C_DBG("I2C Bus error, stopped all operating.");
		i2c->SR1 &=~ I2C_SR1_BERR;
	}
	else if(sr1reg & I2C_SR1_AF){
		I2C_DBG("I2C ACK failure, stopped all operating.");
		i2c->SR1 &=~ I2C_SR1_BERR;
	}
	else if(sr1reg & I2C_SR1_OVR){
		I2C_DBG("I2C Overrun/Underrun error, stopped all operating.");
		i2c->SR1 &=~ I2C_SR1_OVR;
	}
	else if(sr1reg & I2C_SR1_TIMEOUT){
		I2C_DBG("I2C TimeOut, stopped all operating.");
		i2c->SR1 &=~ I2C_SR1_TIMEOUT;
	}

	if(conf->controller == I2C_MODE_MASTER) pi2c->stop();

	pi2c->event_handle(I2C_EVENT_ERROR);
	pi2c->abort_it();
}

#if PERIPHERAL_DMA_AVAILABLE
static void i2c_txdma_event_handler(dma_event_t event, void *param){
	i2c_t pi2c = (i2c_t)param;
	dma_t txdma = pi2c->get_txdma();

	if(event == DMA_EVENT_TRANFER_COMPLETE){
		dma_config_t *dma_conf = txdma->get_config();
		if(dma_conf->mode != DMA_MODE_CIRCULAR) pi2c->txdma_stop();

		pi2c->event_handle(I2C_EVENT_TRANSMIT_COMPLETE);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		pi2c->txdma_stop();
		pi2c->event_handle(I2C_EVENT_ERROR);
	}
	xSemaphoreGiveFromISR(pi2c->get_mutex(), NULL);
}
static void i2c_rxdma_event_handler(dma_event_t event, void *param){
	i2c_t pi2c = (i2c_t)param;
	dma_t rxdma = pi2c->get_rxdma();

	if(event == DMA_EVENT_TRANFER_COMPLETE){
		dma_config_t *dma_conf = rxdma->get_config();
		pi2c->rxinfo.count = pi2c->rxinfo.length;

		if(dma_conf->mode != DMA_MODE_CIRCULAR) pi2c->rxdma_stop();
		pi2c->event_handle(I2C_EVENT_RECEIVE_COMPLETE);
	}
	if(event == DMA_EVENT_TRANFER_ERROR){
		pi2c->rxdma_stop();
		pi2c->event_handle(I2C_EVENT_ERROR);
	}
	xSemaphoreGiveFromISR(pi2c->get_mutex(), NULL);
}
#endif /* PERIPHERAL_DMA_AVAILABLE */

#if defined(I2C1)
static i2c i2c_1(I2C1);
i2c_t i2c1 = &i2c_1;
void I2C1_EV_IRQHandler(void){
	I2CCommonEvent_IRQHandler(i2c1);
}
void I2C1_ER_IRQHandler(void){
	I2CCommonError_IRQHandler(i2c1);
}
#endif /* defined(I2C1) */
#if defined(I2C2)
static i2c i2c_2(I2C2);
i2c_t i2c2 = &i2c_2;
void I2C2_EV_IRQHandler(void){
	I2CCommonEvent_IRQHandler(i2c2);
}
void I2C2_ER_IRQHandler(void){
	I2CCommonError_IRQHandler(i2c2);
}
#endif /* defined(I2C2) */
#if defined(I2C3)
static i2c i2c_3(I2C3);
i2c_t i2c3 = &i2c_3;
void I2C3_EV_IRQHandler(void){
	I2CCommonEvent_IRQHandler(i2c3);
}
void I2C3_ER_IRQHandler(void){
	I2CCommonError_IRQHandler(i2c3);
}
#endif /* defined(I2C3) */


#endif /* PERIPHERAL_I2C_AVAILABLE */


/*
 * dma.cpp
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */
#include "drivers/dmac.h"
#if PERIPHERAL_DMAC_AVAILABLE

#include "drivers/system.h"
#include "common/bits_check.h"
#include "common/macro.h"
#include "common/log_monitor.h"





static const char *TAG = "DMA";
static const uint8_t Channel_Index[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};


#ifdef CONFIG_PERIPH_DMAC_LOG
#define DMAC_DBG(__MSG__)\
	LOG_MESS(LOG_ERROR, TAG, __MSG__);
#else
#define DMAC_DBG(__MSG__)\
{}
#endif


static void DMACommon_IRQ_Handler(DMA_Stream_TypeDef *pstream, dmac_t pdma);


dmac::dmac(DMA_TypeDef *dma){
	_instance = dma;
}

err_t dmac::initialize(dmac_config_t *conf){
	err_t ret;
	__IO uint32_t tmpreg;

	_conf = conf;
#if defined(STM32F1)
#if defined(DMA1)
	if(_instance == DMA1) SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
#endif /* defined(DMA1) */
#if defined(DMA2)
	if(_instance == DMA2) SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);
#endif /* defined(DMA2) */
	CLEAR_BIT(_conf->stream->CCR, DMA_CCR_EN);
	ret = wait_bits(&(_conf->stream->CCR), DMA_CCR_EN, BITS_RESET, 500U);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	tmpreg = READ_REG(_conf->stream->CCR);
	tmpreg &=~ 0xFFFFU;

	tmpreg |= (_conf->mode << DMA_CCR_CIRC_Pos) | (_conf->channelpriority << DMA_CCR_PL_Pos)
			 | DMA_CCR_MINC | (_conf->datasize << DMA_CCR_PSIZE_Pos) | (_conf->datasize << DMA_CCR_MSIZE_Pos);
	if(_conf->direction == DMA_MEM_TO_MEM) tmpreg |= DMA_CCR_MEM2MEM;
	else tmpreg |= (_conf->direction << DMA_CCR_DIR_Pos);

	WRITE_REG(_conf->stream->CCR, tmpreg);

#if defined(DMA1)
	if(_instance == DMA1){
		_IRQn = (IRQn_Type)(_conf->channel + DMA1_Channel1_IRQn);
	}
#endif /* DMA1 */
#if defined(DMA2)
	if(_instance == DMA2){
		_IRQn = (IRQn_Type)(_conf->channel + DMA2_Channel1_IRQn);
	}
#endif /* DMA */
/** -------------------------------------------------------------------------------------------------- */
#elif defined(STM32F4)
#if defined(DMA1)
	if(_instance == DMA1) SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
#endif /* defined(DMA1) */
#if defined(DMA2)
	if(_instance == DMA2) SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
#endif /* defined(DMA2) */
	CLEAR_BIT(_conf->stream->CR, DMA_SxCR_EN);
	ret = wait_bits(&(_conf->stream->CR), DMA_SxCR_EN, BITS_RESET, 500U);
	IS_ERROR(ret){
		ERROR_CAPTURE(ret);
		return ret;
	}

	tmpreg = READ_REG(_conf->stream->CR);
	tmpreg &= ~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST |
				DMA_SxCR_CT    | DMA_SxCR_DBM    | DMA_SxCR_PL     |
			    DMA_SxCR_MSIZE | DMA_SxCR_PSIZE  | DMA_SxCR_MINC   |
			    DMA_SxCR_PINC  | DMA_SxCR_CIRC   | DMA_SxCR_DIR    |
			    DMA_SxCR_PFCTRL| DMA_SxCR_TCIE   | DMA_SxCR_HTIE   |
				DMA_SxCR_TEIE  | DMA_SxCR_DMEIE  | DMA_SxCR_EN     );

	tmpreg |= (uint32_t)((_conf->channel << DMA_SxCR_CHSEL_Pos) | (_conf->channelpriority << DMA_SxCR_PL_Pos) |
						 (_conf->datasize << DMA_SxCR_PSIZE_Pos) | (_conf->datasize << DMA_SxCR_MSIZE_Pos)  |
						 DMA_SxCR_MINC | (_conf->mode << DMA_SxCR_CIRC_Pos) | (_conf->direction << DMA_SxCR_DIR_Pos));

	if(_conf->fifo == DMAC_FIF0) {tmpreg |= (_conf->burst << DMA_SxCR_MBURST_Pos) | (_conf->burst << DMA_SxCR_PBURST_Pos);}
	WRITE_REG(_conf->stream->CR, tmpreg);

	tmpreg = READ_REG(_conf->stream->FCR);
	tmpreg &=~ (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
	tmpreg |= _conf->fifo;
	if(_conf->fifo == DMAC_FIF0) tmpreg |= DMA_SxFCR_FTH;
	WRITE_REG(_conf->stream->FCR, tmpreg);

	_stream = (((uint32_t)_conf->stream & 0xFFU) - 16U) / 24U;
	_int_index = Channel_Index[_stream];

	clear_all_flag();

	if(_instance == DMA1){
		if(_stream == 7U) _IRQn = DMA1_Stream7_IRQn;
		else _IRQn = (IRQn_Type)(_stream + 11U);
	}
	else if(_instance == DMA2){
		if(_stream > 4U) _IRQn = (IRQn_Type)(_stream + 63U);
		else _IRQn = (IRQn_Type)(_stream + 56U);
	}
#endif /* STM32F4 */

	if(_conf->interruptpriority + CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY > 15){
		ERROR_SET(ret, E_INVALID);
		DMAC_DBG("Interrupt priority out of range");
		system_reset();
		return ret;
	}
	_conf->interruptpriority += CONFIG_RTOS_MAX_SYSTEM_INT_PRIORITY;
	__NVIC_SetPriority(_IRQn, _conf->interruptpriority);
	_mutex = xSemaphoreCreateMutex();

	return ret;
}

void dmac::deinitialize(void){
#if defined(STM32F1)
#if defined(DMA1)
	if(_instance == DMA1) CLEAR_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
#endif /* defined(DMA1) */
#if defined(DMA2)
	if(_instance == DMA2) CLEAR_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);
#endif /* defined(DMA2) */
	CLEAR_REG(_conf->stream->CCR);
	CLEAR_REG(_conf->stream->CMAR);
	CLEAR_REG(_conf->stream->CNDTR);
	CLEAR_REG(_conf->stream->CPAR);
#elif defined(STM32F4)
#if defined(DMA1)
	if(_instance == DMA1) CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
#endif /* defined(DMA1) */
#if defined(DMA2)
	if(_instance == DMA2) CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
#endif /* defined(DMA2) */

	CLEAR_REG(_conf->stream->CR);
	CLEAR_REG(_conf->stream->NDTR);
	CLEAR_REG(_conf->stream->PAR);
	CLEAR_REG(_conf->stream->M0AR);
	CLEAR_REG(_conf->stream->M1AR);
	WRITE_REG(_conf->stream->FCR, 0x21UL);
#endif
	clear_all_flag();

    xSemaphoreTake(_mutex, portMAX_DELAY);
    vSemaphoreDelete(_mutex);
}

DMA_TypeDef *dmac::get_instance(void){
	return _instance;
}

dmac_config_t *dmac::get_config(void){
	return _conf;
}

uint16_t dmac::get_transfer_counter(void){
#if defined(STM32F1)
	return READ_REG(_conf->stream->CNDTR);
#elif defined(STM32F4)
	return READ_REG(_conf->stream->NDTR);
#endif /* STM32F4 */
}

SemaphoreHandle_t dmac::get_mutex(void){
	return _mutex;
}


void dmac::register_event_handler(dmac_event_handler_f event_handler_function, void *event_parameter){
	_event_handler = event_handler_function;
	_event_parameter = event_parameter;
}

void dmac::unregister_event_handler(void){
	_event_handler = NULL;
	_event_parameter = NULL;
}

void dmac::event_handle(dmac_event_t event){
	if(_event_handler) _event_handler(event, _event_parameter);
}

void dmac::set_direction(dmac_direction_t direction){
	__IO uint32_t tmpreg;

	_conf->direction = direction;
	CLEAR_BIT(_conf->stream->CR, DMA_SxCR_EN);
#if defined(STM32F1)
	tmpreg = READ_REG(_conf->stream->CCR);
	tmpreg &=~ (DMA_CCR_MEM2MEM | DMA_CCR_DIR);
	if(_conf->direction == DMA_MEM_TO_MEM) tmpreg |= DMA_CCR_MEM2MEM;
	else tmpreg |= (_conf->direction << DMA_CCR_DIR_Pos);
	WRITE_REG(_conf->stream->CCR, tmpreg);
#elif defined(STM32F4)
	tmpreg = READ_REG(_conf->stream->CR);
	tmpreg &=~ DMA_SxCR_DIR;
	tmpreg |= (_conf->direction << DMA_SxCR_DIR_Pos);
	WRITE_REG(_conf->stream->CR, tmpreg);
#endif
}

void dmac::set_datasize(dmac_datasize_t datasize){
	__IO uint32_t tmpreg;

	_conf->datasize = datasize;
	CLEAR_BIT(_conf->stream->CR, DMA_SxCR_EN);
#if defined(STM32F1)
	tmpreg = READ_REG(_conf->stream->CCR);
	tmpreg &=~ (DMA_CCR_PSIZE | DMA_CCR_PSIZE);
	tmpreg |= (_conf->datasize << DMA_CCR_PSIZE_Pos) | (_conf->datasize << DMA_CCR_MSIZE_Pos);
	WRITE_REG(_conf->stream->CCR, tmpreg);
#elif defined(STM32F4)
	tmpreg = READ_REG(_conf->stream->CR);
	tmpreg &=~ (DMA_SxCR_PSIZE | DMA_SxCR_PSIZE);
	tmpreg |= (_conf->datasize << DMA_SxCR_PSIZE_Pos) | (_conf->datasize << DMA_SxCR_MSIZE_Pos);
	WRITE_REG(_conf->stream->CR, tmpreg);
#endif
}


void dmac::config(uint32_t psource, uint32_t pdest, uint32_t size){
#if defined(STM32F1)
	_conf->stream->CNDTR = size;
	if(_conf->direction == DMAC_MEM_TO_PERIPH){
		_conf->stream->CMAR = psource;
		_conf->stream->CPAR = pdest;
	}
	else{
		_conf->stream->CMAR = pdest;
		_conf->stream->CPAR = psource;
	}
#elif defined(STM32F4)
	_conf->stream->NDTR = size;
	if(_conf->direction == DMAC_MEM_TO_PERIPH){
		_conf->stream->PAR = pdest;
		_conf->stream->M0AR = psource;
	}
	else{
		_conf->stream->PAR = psource;
		_conf->stream->M0AR = pdest;
	}
#endif
}


err_t dmac::config_start_session(struct dmac_session_config_t conf){
	err_t ret;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, conf.timeout));
	if(err == pdTRUE){
#if defined(STM32F1)
		CLEAR_BIT(_conf->stream->CCR, DMA_CCR_EN);
#elif defined(STM32F4)
		CLEAR_BIT(_conf->stream->CR, DMA_SxCR_EN);
#endif

		config(conf.psource, conf.pdest, conf.xfersize);
		clear_all_flag();

#if defined(STM32F1)
		if(conf.interrupt_enable == true)
			_conf->stream->CCR |= _conf->interruptoption;

		_conf->stream->CCR |= DMA_CCR_EN;
#elif defined(STM32F4)
		if(conf.interrupt_enable == true)
			SET_BIT(_conf->stream->CR, _conf->interruptoption | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

		SET_BIT(_conf->stream->CR, DMA_SxCR_EN);
#endif
		if(conf.interrupt_enable == true){
			__NVIC_ClearPendingIRQ(_IRQn);
			__NVIC_EnableIRQ(_IRQn);
		}
	}
	else{
		ERROR_SET(ret, E_BUSY);
		DMAC_DBG("DMA session was blocked by mutex");
	}

	return ret;
}

void dmac::stop_session(void){
#if defined(STM32F1)
	_conf->stream->CCR &=~ (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
	_conf->stream->CCR &=~ (DMA_CCR_EN);
#elif defined(STM32F4)
	CLEAR_BIT(_conf->stream->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
	CLEAR_BIT(_conf->stream->FCR, DMA_SxFCR_FEIE);
	CLEAR_BIT(_conf->stream->CR, DMA_SxCR_EN);
	CLEAR_REG(_conf->stream->PAR);
	CLEAR_REG(_conf->stream->M0AR);
	CLEAR_REG(_conf->stream->M1AR);
#endif /* STM32F4 */
	clear_all_flag();
	__NVIC_ClearPendingIRQ(_IRQn);
	__NVIC_DisableIRQ(_IRQn);
}


err_t dmac::wait_for(dmac_event_t break_event, uint32_t timeout){
	err_t ret;
	__IO uint32_t PollValue = 0U, tick, isr;
	BaseType_t err, in_it = xPortIsInsideInterrupt();

	(in_it == pdTRUE)? (err = xSemaphoreTakeFromISR(_mutex, NULL)) : (err = xSemaphoreTake(_mutex, 1));
	if(err == pdTRUE){
		ERROR_SET(ret, E_FAIL);
		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
		DMAC_DBG("DMA session has not been configured and started");
		return ret;
	}

#if defined(STM32F1)
	if(_conf->stream->CCR & DMA_CCR_CIRC){
#elif defined(STM32F4)
	if(READ_BIT(_conf->stream->CR, DMA_SxCR_CIRC)){
#endif /* STM32F4 */
		ERROR_SET(ret, E_NOT_ALLOWED);
		return ret;
	}

#if defined(STM32F1)
	if(break_event == DMAC_EVENT_TRANFER_COMPLETE)
		PollValue = (uint32_t)(0x02U << (_conf->channel));
	else if(break_event == DMAC_EVENT_HALF_TRANFER)
		PollValue = (uint32_t)(0x04U << (_conf->channel));
#elif defined(STM32F4)
	if(break_event == DMAC_EVENT_TRANFER_COMPLETE)
		PollValue = (uint32_t)(0x20U << _int_index);
	else if(break_event == DMAC_EVENT_HALF_TRANFER)
		PollValue = (uint32_t)(0x10U << _int_index);
#endif /* STM32F4 */
	else{
		ERROR_SET(ret, E_NOT_ALLOWED);
		return ret;
	}

	tick = get_tick();
#if defined(STM32F1)
	__IO uint32_t TE_SR_Value = (uint32_t)(0x08U << (_conf->channel));
	isr = get_ISR();
	while(!(isr & PollValue)){
		if(get_tick() - tick > timeout){
			ERROR_SET(ret, E_TIMEOUT);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
			stop_session();
			return ret;
		}

		isr = get_ISR();
		if(isr & TE_SR_Value){
			clear_all_flag();
			ERROR_SET(ret, E_FAIL);
			(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
			stop_session();
			return ret;
		}
	}

	if(break_event == DMAC_EVENT_HALF_TRANFER){
		clear_IFCR((uint32_t)(0x04U << (_conf->channel)));
	}
	else if(break_event == DMAC_EVENT_TRANFER_COMPLETE){
		clear_IFCR((uint32_t)(0x02U << (_conf->channel)));
	}


#elif defined(STM32F4)
	isr = get_ISR();
	while(!(isr & PollValue)){
		if(timeout != NO_WAIT){
			if(get_tick() - tick > timeout){
				ERROR_SET(ret, E_TIMEOUT);
				(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
				stop_session();
				return ret;
			}
		}
		isr = get_ISR();
		if(isr & (0x01U << _int_index)){ // FEIE.
			clear_IFCR(0x01U << _int_index);
			break;
		}
		if(isr & (0x04U << _int_index)){ // DMEIE.
			clear_IFCR(0x04U << _int_index);
			break;
		}
		if(isr & (0x08U << _int_index)){ // TEIE.
			clear_IFCR(0x08U << _int_index);
			break;
		}
	}

	if(isr & (0x08U << _int_index)){
		stop_session();
		clear_IFCR((DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << _int_index);
		ERROR_SET(ret, E_FAIL);
		(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);
		return ret;
	}
	if(break_event == DMAC_EVENT_TRANFER_COMPLETE)
		clear_IFCR((DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << _int_index);
	else
		clear_IFCR(DMA_LIFCR_CHTIF0 << _int_index);

#endif /* STM32F4 */
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(_mutex, NULL) : xSemaphoreGive(_mutex);

	return ret;
}


void dmac::clear_IFCR(__IO uint32_t Value){
#if defined(STM32F1)
	_instance->IFCR = Value;
#elif defined(STM32F4)
	(_stream < 4)? WRITE_REG(_instance->LIFCR, Value) : WRITE_REG(_instance->HIFCR, Value);
#endif /* STM32F4 */
}

void dmac::clear_all_flag(void){
#if defined(STM32F1)
	WRITE_REG(_instance->IFCR, (DMA_ISR_GIF1 << (_conf->channel * 4U)));
#elif defined(STM32F4)
	clear_IFCR(0x3FU << _int_index);
#endif /* STM32F4 */
}

__IO uint32_t dmac::get_ISR(void){
	__IO uint32_t isr = 0;

#if defined(STM32F1)
	isr = READ_REG(_instance->ISR);
#elif defined(STM32F4)
	(_stream < 4)? isr = READ_REG(_instance->LISR) : isr = READ_REG(_instance->HISR);
#endif /* STM32F4 */

	return isr;
}



#if defined(STM32F1)
static void DMACommon_IRQ_Handler(DMA_Stream_TypeDef *pstream, dmac_t pdma){
	BaseType_t in_it = xPortIsInsideInterrupt();
	dmac_event_t event = DMAC_EVENT_NOEVENT;
	DMA_TypeDef *dma = pdma->get_instance();
	uint8_t index = ((uint32_t)pstream - (uint32_t)dma - 0x008U)/0x14U;

	if(READ_BIT(pstream->CCR, DMA_CCR_HTIE) && READ_BIT(dma->ISR, (DMA_ISR_HTIF1 << (index*4)))){
		if(!READ_BIT(pstream->CCR, DMA_CCR_CIRC)) CLEAR_BIT(pstream->CCR, DMA_CCR_HTIE);
		WRITE_REG(dma->IFCR, (DMA_IFCR_CHTIF1 << (index*4)));
		event = DMAC_EVENT_HALF_TRANFER;
		goto event_handle;
	}
	else if(READ_BIT(pstream->CCR, DMA_CCR_TCIE) && READ_BIT(dma->ISR, (DMA_ISR_TCIF1 << (index*4)))){
		if(!READ_BIT(pstream->CCR, DMA_CCR_CIRC)) CLEAR_BIT(pstream->CCR, DMA_CCR_TEIE | DMA_CCR_TCIE);
		WRITE_REG(dma->IFCR, (DMA_IFCR_CTCIF1 << (index*4)));
		event = DMAC_EVENT_TRANFER_COMPLETE;
		goto event_handle;
	}
	else if(READ_BIT(pstream->CCR, DMA_CCR_TEIE) && READ_BIT(dma->ISR, (DMA_ISR_TEIF1 << (index*4)))){
		CLEAR_BIT(pstream->CCR, DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		WRITE_REG(dma->IFCR, (DMA_IFCR_CGIF1 << (index*4)));
		event = DMAC_EVENT_TRANFER_ERROR;
		goto event_handle;
	}

	event_handle:
	pdma->event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(pdma->get_mutex(), NULL) : xSemaphoreGive(pdma->get_mutex());
}

/** -------------------------------------------------------------------------------------------------- **/
#elif defined(STM32F4)
static void DMACommon_IRQ_Handler(DMA_Stream_TypeDef *pstream, dmac_t pdma){
	BaseType_t in_it = xPortIsInsideInterrupt();
	uint8_t num_stream = (((uint32_t)pstream & 0xFFU) - 16U) / 24U;
	uint8_t index = Channel_Index[num_stream];
	DMA_TypeDef *dma = pdma->get_instance();
	dmac_event_t event = DMAC_EVENT_NOEVENT;

	if((num_stream < 4)? READ_BIT(dma->LISR, (DMA_LISR_HTIF0 << index)) : READ_BIT(dma->HISR, (DMA_HISR_HTIF4 << index))){
		(num_stream < 4)? WRITE_REG(dma->LIFCR, (DMA_LIFCR_CHTIF0 << index)) : WRITE_REG(dma->HIFCR, (DMA_HIFCR_CHTIF4 << index));
		if(!READ_BIT(pstream->CR, DMA_SxCR_CIRC))
			CLEAR_BIT(pstream->CR, DMA_SxCR_HTIE);
		event = DMAC_EVENT_HALF_TRANFER;
		goto event_handle;
	}

	if((num_stream < 4)? READ_BIT(dma->LISR, (DMA_LISR_TCIF0 << index)) : READ_BIT(dma->HISR, (DMA_HISR_TCIF4 << index))){
		(num_stream < 4)? WRITE_REG(dma->LIFCR, (0x3FU << index)) : WRITE_REG(dma->HIFCR, (0x3FU << index));
		if(!READ_BIT(pstream->CR, DMA_SxCR_CIRC)){
			CLEAR_BIT(pstream->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
			CLEAR_BIT(pstream->FCR, DMA_SxFCR_FEIE);
		}
		event = DMAC_EVENT_TRANFER_COMPLETE;
		goto event_handle;
	}

	if((num_stream < 4)? READ_BIT(dma->LISR, (DMA_LISR_TEIF0 << index)) : READ_BIT(dma->HISR, (DMA_HISR_TEIF4 << index))){
		(num_stream < 4)? WRITE_REG(dma->LIFCR, (DMA_LIFCR_CTEIF0 << index)) : WRITE_REG(dma->HIFCR, (DMA_HIFCR_CTEIF4 << index));
		CLEAR_BIT(pstream->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
		CLEAR_BIT(pstream->FCR, DMA_SxFCR_FEIE);
		event = DMAC_EVENT_TRANFER_ERROR;
		goto event_handle;
	}

	event_handle:
	pdma->event_handle(event);
	(in_it == pdTRUE)? xSemaphoreGiveFromISR(pdma->get_mutex(), NULL) : xSemaphoreGive(pdma->get_mutex());
}
#endif /* STM32F4 */







#if defined(DMA1_Stream0)
static dmac dma1_0(DMA1);
dmac_t dma1_stream0 = &dma1_0;
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
static dmac dma1_1(DMA1);
dmac_t dma1_stream1 = &dma1_1;
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
static dmac dma1_2(DMA1);
dmac_t dma1_stream2 = &dma1_2;
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
static dmac dma1_3(DMA1);
dmac_t dma1_stream3 = &dma1_3;
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
static dmac dma1_4(DMA1);
dmac_t dma1_stream4 = &dma1_4;
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
static dmac dma1_5(DMA1);
dmac_t dma1_stream5 = &dma1_5;
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
static dmac dma1_6(DMA1);
dmac_t dma1_stream6 = &dma1_6;
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
static dmac dma1_7(DMA1);
dmac_t dma1_stream7 = &dma1_7;
#endif /* defined(DMA1_Stream7) */
/**
 *  DMA2 IRQ HANDLER
 */
#if defined(DMA2_Stream0)
static dmac dma2_0(DMA2);
dmac_t dma2_stream0 = &dma2_0;
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
static dmac dma2_1(DMA2);
dmac_t dma2_stream1 = &dma2_1;
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
static dmac dma2_2(DMA2);
dmac_t dma2_stream2 = &dma2_2;
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
static dmac dma2_3(DMA2);
dmac_t dma2_stream3 = &dma2_3;
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
static dmac dma2_4(DMA2);
dmac_t dma2_stream4 = &dma2_4;
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
static dmac dma2_5(DMA2);
dmac_t dma2_stream5 = &dma2_5;
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
static dmac dma2_6(DMA2);
dmac_t dma2_stream6 = &dma2_6;
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
static dmac dma2_7(DMA2);
dmac_t dma2_stream7 = &dma2_7;
#endif /* defined(DMA2_Stream7) */

/**
 * STM32F1 Device
 * DMA interrupt handler by channel.
 */
#if defined(STM32F1)
/**
 *  DMA1 IRQ HANDLER
 */
#if defined(DMA1_Channel1)
void DMA1_Channel1_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream1, dma1_stream1);
}
#endif /* DMA1_Channel1 */
#if defined(DMA1_Channel2)
void DMA1_Channel2_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream2, dma1_stream2);
}
#endif /* DMA1_Channel2 */
#if defined(DMA1_Channel3)
void DMA1_Channel3_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream3, dma1_stream3);
}
#endif /* DMA1_Channel3 */
#if defined(DMA1_Channel4)
void DMA1_Channel4_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream4, dma1_stream4);
}
#endif /* DMA1_Channel4 */
#if defined(DMA1_Channel5)
void DMA1_Channel5_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream5, dma1_stream5);
}
#endif /* DMA1_Channel5 */
#if defined(DMA1_Channel6)
void DMA1_Channel6_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream6, dma1_stream6);
}
#endif /* DMA1_Channel6 */
#if defined(DMA1_Channel7)
void DMA1_Channel7_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream7, dma1_stream7);
}
#endif /* DMA1_Channel4 */
/**
 *  DMA2 IRQ HANDLER
 */
#if defined(DMA2_Channel1)
void DMA2_Channel1_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream1, dma2_stream1);
}
#endif /* DMA2_Channel1 */
#if defined(DMA2_Channel2)
void DMA2_Channel2_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream2, dma2_stream2);
}
#endif /* DMA2_Channel2 */
#if defined(DMA2_Channel3)
void DMA2_Channel3_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream3, dma2_stream3);
}
#endif /* DMA2_Channel3 */
#if defined(DMA2_Channel4)
void DMA2_Channel4_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream4, dma2_stream4);
}
#endif /* DMA2_Channel4 */
#if defined(DMA2_Channel5)
void DMA2_Channel5_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream5, dma2_stream5);
}
#endif /* DMA2_Channel5 */
#if defined(DMA2_Channel6)
void DMA2_Channel6_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream6, dma2_stream6);
}
#endif /* DMA2_Channel6 */
#if defined(DMA2_Channel7)
void DMA2_Channel7_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream7, dma2_stream7);
}
#endif /* DMA2_Channel4 */

#endif /* STM32F1 */


/**
 * STM32F4 Device
 * DMA interrupt handler by channel in stream.
 */
#if defined(STM32F4)
/**
 *  DMA1 IRQ HANDLER
 */
#if defined(DMA1_Stream0)
void DMA1_Stream0_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream0, dma1_stream0);
}
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
void DMA1_Stream1_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream1, dma1_stream1);
}
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
void DMA1_Stream2_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream2, dma1_stream2);
}
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
void DMA1_Stream3_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream3, dma1_stream3);
}
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
void DMA1_Stream4_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream4, dma1_stream4);
}
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
void DMA1_Stream5_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream5, dma1_stream5);
}
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
void DMA1_Stream6_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream6, dma1_stream6);
}
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
void DMA1_Stream7_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA1_Stream7, dma1_stream7);
}
#endif /* defined(DMA1_Stream7) */
/**
 *  DMA2 IRQ HANDLER
 */
#if defined(DMA2_Stream0)
void DMA2_Stream0_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream0, dma2_stream0);
}
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
void DMA2_Stream1_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream1, dma2_stream1);
}
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
void DMA2_Stream2_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream2, dma2_stream2);
}
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
void DMA2_Stream3_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream3, dma2_stream3);
}
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
void DMA2_Stream4_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream4, dma2_stream4);
}
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
void DMA2_Stream5_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream5, dma2_stream5);
}
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
void DMA2_Stream6_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream6, dma2_stream6);
}
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
void DMA2_Stream7_IRQHandler(void){
	DMACommon_IRQ_Handler(DMA2_Stream7, dma2_stream7);
}
#endif /* defined(DMA2_Stream7) */

#endif /* STM32F4 */




#endif /* PERIPHERAL_DMAC_AVAILABLE */



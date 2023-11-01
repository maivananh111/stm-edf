/*
 * i2c.h
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_I2C_H_
#define PERIPHERALS_I2C_H_


#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_I2C_EN && (defined(I2C1) || defined(I2C2) || defined(I2C3) || defined(I2C4) || defined(I2C5) || defined(I2C6))
#define PERIPHERAL_I2C_AVAILABLE 1

#include "common/error_check.h"
#include "drivers/gpio.h"
#if CONFIG_PERIPH_DMAC_EN
#include "drivers/dmac.h"
#endif /* CONFIG_PERIPH_DMAC_EN */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	I2C_MODE_MASTER,
	I2C_MODE_SLAVE,
} i2c_mode_t;

typedef enum{
	I2C_STANDARD_MODE = 0,
	I2C_FAST_MODE,
} i2c_speedmode_t;

typedef enum{
	I2C_WRITE = 0,
	I2C_READ,
} i2c_action_t;

typedef enum{
	I2C_EVENT_NOEVENT,
	I2C_EVENT_TRANSMIT_COMPLETE,
	I2C_EVENT_RECEIVE_COMPLETE,
	I2C_EVENT_ERROR,
} i2c_event_t;

typedef void (*i2c_evcb_t)(i2c_event_t, void *);

typedef struct{
	i2c_mode_t		   controller = I2C_MODE_MASTER;
	i2c_speedmode_t     speedmode = I2C_STANDARD_MODE;
	uint32_t 		    frequency = 100000U;
	uint32_t    interruptpriority = 0;
	uint8_t     slavemode_address = 0U;
	bool 			  generalcall = false;
#if CONFIG_PERIPH_I2C_DEFAULT_ANALOG_FILTER
	bool             analogfilter = true;
#else
	bool             analogfilter = false;
#endif /* CONFIG_PERIPH_I2C_DEFAULT_ANALOG_FILTER */
	uint8_t    digitalnoisefilter = CONFIG_PERIPH_I2C_DEFAULT_DIGITAL_FILTER;

	char 		          *sclpin = NULL;
	char   		          *sdapin = NULL;
#if defined(STM32F1)
	bool               gpio_remap = false;
#endif /* STM32F1 */
} i2c_config_t;

struct i2c_xfer_t{
	uint8_t *buffer;
	uint16_t length;
	uint16_t count;
};


using i2c_event_handler_f = std::function<void(i2c_event_t, void *)>;

class i2c{
	public:
	/** Constructor */
		i2c(I2C_TypeDef *i2c);
	/** Initialize/deinitialize */
		err_t initialize(i2c_config_t *conf);
		void deinitialize(void);
	/** Event handle */
		void register_event_handler(i2c_event_handler_f event_handler_function, void *param = NULL);
		void unregister_event_handler(void);
		void event_handle(i2c_event_t event);
	/** DMAC link */
#if PERIPHERAL_DMAC_AVAILABLE
		void link_dmac(dmac_t txdmac = NULL, dmac_t rxdmac = NULL);
		void unlink_dmac(void);
#endif /* PERIPHERAL_DMAC_AVAILABLE */
	/** Get parameter */
		I2C_TypeDef *get_instance(void);
		i2c_config_t *get_config(void);
		IRQn_Type get_ev_irq(void);
		IRQn_Type get_er_irq(void);
#if PERIPHERAL_DMAC_AVAILABLE
		dmac_t get_txdmac(void);
		dmac_t get_rxdmac(void);
#endif /* PERIPHERAL_DMAC_AVAILABLE */
		SemaphoreHandle_t get_mutex(void); /** I2C is half duplex communication. */
		i2c_action_t get_current_action(void);
		uint8_t get_address(void);

	/** I2C operation */
		err_t is_device_ready(uint8_t address, uint8_t num_trials, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);


		err_t transmit(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);
		err_t receive(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);

		err_t transmit_it(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_it(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);

#if PERIPHERAL_DMAC_AVAILABLE
		err_t transmit_dmac(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);
		err_t receive_dmac(uint8_t address, uint8_t *pdata, uint16_t data_size, uint16_t timeout = CONFIG_PERIPH_I2C_DEFAULT_OPERATION_TIMEOUT);
#endif /* PERIPHERAL_DMAC_AVAILABLE */

		void abort_it(void);
		void abort_dmac(void);

		err_t start(void);
		void stop(void);
		void send_address(uint8_t address, i2c_action_t action);

#if PERIPHERAL_DMAC_AVAILABLE
		err_t txdmac_stop(void);
		err_t rxdmac_stop(void);
		void txdmac_event_handler(dmac_event_t event, void *param);
		void rxdmac_event_handler(dmac_event_t event, void *param);
#endif /* PERIPHERAL_DMAC_AVAILABLE */
		void setup_reception(void);


		i2c_xfer_t txinfo, rxinfo;



	private:
		I2C_TypeDef *_instance;
		i2c_config_t *_conf;
		gpio_pin_t _sclpin, _sdapin;
		IRQn_Type _evIRQn, _erIRQn;

		SemaphoreHandle_t _mutex;

#if PERIPHERAL_DMAC_AVAILABLE
		dmac_t _txdmac = NULL, _rxdmac = NULL;
		dmac_config_t *_txdmac_conf = NULL, *_rxdmac_conf = NULL;
#endif /* PERIPHERAL_DMAC_AVAILABLE */
		void *_event_parameter = NULL;
		i2c_event_handler_f _event_handler = NULL;

		uint8_t _current_dev_address;
		i2c_action_t _action = I2C_WRITE;

		err_t hardware_initialize(void);
		void hardware_deinitialize(void);

		void ACK_failure_handle(void);
		void clear_ADDR_flag(void);
		err_t wait_for_ready(void);
		err_t wait_addr_flag(void);
};

typedef i2c* i2c_t;

i2c_t i2c_str_decode(char *str);

#if defined(I2C1)
extern i2c_t i2c1;
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
#endif /* defined(I2C1) */
#if defined(I2C2)
extern i2c_t i2c2;
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
#endif /* defined(I2C2) */
#if defined(I2C3)
extern i2c_t i2c3;
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
#endif /* defined(I2C3) */

#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_I2C_AVAILABLE 0
#endif /* CONFIG_PERIPH_I2C_EN && (defined(I2C1) || defined(I2C2) || defined(I2C3) || defined(I2C4) || defined(I2C5) || defined(I2C6)) */

#endif /* PERIPHERALS_DRV_I2C_H_ */

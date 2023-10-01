/*
 * fmc_sdram.cpp
 *
 *  Created on: Mar 22, 2023
 *      Author: anh
 */

#include "drivers/fmc_sdram.h"
#if PERIPHERAL_FMC_SDRAM_AVAILABLE

#include "common/bits_check.h"
#include "drivers/gpio.h"
#include "drivers/system.h"


#define SDRAM_INSTANCE FMC_Bank5_6
#define FMC_SDRAM_TIMEOUT CONFIG_FMC_SDRAM_DEFAULT_TIMEOUT

#define FMC_SDRAM_CMD_NORMAL_MODE               (0x00000000U)
#define FMC_SDRAM_CMD_CLK_ENABLE                (0x00000001U)
#define FMC_SDRAM_CMD_PALL                      (0x00000002U)
#define FMC_SDRAM_CMD_AUTOREFRESH_MODE          (0x00000003U)
#define FMC_SDRAM_CMD_LOAD_MODE                 (0x00000004U)
#define FMC_SDRAM_CMD_SELFREFRESH_MODE          (0x00000005U)
#define FMC_SDRAM_CMD_POWERDOWN_MODE            (0x00000006U)

#define FMC_SDRAM_CMD_TARGET_BANK2              FMC_SDCMR_CTB2
#define FMC_SDRAM_CMD_TARGET_BANK1              FMC_SDCMR_CTB1
#define FMC_SDRAM_CMD_TARGET_BANK1_2            (0x00000018U)

static sdram_config_t *_conf;
static void fmc_sdram_hardware_init(void);
static void sdram_param_init(void);

void fmc_sdram_init(sdram_config_t *conf){
	_conf = conf;

	fmc_sdram_hardware_init();

	SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

	if(_conf->bank == SDRAM_BANK1){
		/**
		 * Configure control register.
		*/
		__IO uint32_t tmpreg = READ_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK1]);
		tmpreg &=~ 0xFFFFU;

		tmpreg |= _conf->clock_cycle | _conf->coladdr_size
				| _conf->rowaddr_size | _conf->data_size
				| _conf->num_banks | _conf->cas_latency
			    | _conf ->read_delay;
		if(_conf->write_protection) tmpreg |= FMC_SDCR1_WP;
		if(_conf->read_burst) tmpreg |= FMC_SDCR1_RBURST;

		WRITE_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK1], tmpreg);

		/**
		 * Configure timing register.
		*/
		tmpreg = READ_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK1]);
		tmpreg &=~ 0xFFFFFFFF;
		tmpreg |= ((_conf->timing.tMRD-1U) << FMC_SDTR1_TMRD_Pos)
				| ((_conf->timing.tXSR-1U) << FMC_SDTR1_TXSR_Pos)
				| ((_conf->timing.tRAS-1U) << FMC_SDTR1_TRAS_Pos)
				| ((_conf->timing.tRC -1U) << FMC_SDTR1_TRC_Pos)
				| ((_conf->timing.tDPL-1U) << FMC_SDTR1_TWR_Pos)
				| ((_conf->timing.tRP -1U) << FMC_SDTR1_TRP_Pos)
				| ((_conf->timing.tRCD-1U) << FMC_SDTR1_TRCD_Pos);

		WRITE_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK1], tmpreg);
	}
	else{
		/**
		 * Configure control register.
		*/
		/* SDCLK, BURST, RPIPE must be configure in SDCR1 register */
		__IO uint32_t tmpreg = READ_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK1]);
		tmpreg &=~ (FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE);

		tmpreg |= _conf->clock_cycle | _conf ->read_delay;
		if(_conf->read_burst) tmpreg |= FMC_SDCR1_RBURST;

		WRITE_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK1], tmpreg);


		/* Configure SDCR2 register */
		tmpreg = READ_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK2]);
		tmpreg &=~ 0xFFFFU;

		tmpreg |= _conf->coladdr_size
				| _conf->rowaddr_size | _conf->data_size
				| _conf->num_banks | _conf->cas_latency;
		if(_conf->write_protection) tmpreg |= FMC_SDCR1_WP;

		WRITE_REG(SDRAM_INSTANCE->SDCR[SDRAM_BANK2], tmpreg);

		/**
		 * Configure timing register.
		*/
		tmpreg = READ_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK1]);
		tmpreg &=~ (FMC_SDTR1_TRC | FMC_SDTR1_TRP);
		tmpreg |= ((_conf->timing.tRC -1U) << FMC_SDTR1_TRC_Pos)
				| ((_conf->timing.tRP -1U) << FMC_SDTR1_TRP_Pos);

		WRITE_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK1], tmpreg);


		tmpreg = READ_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK2]);
		tmpreg &=~ 0xFFFFFFFF;
		tmpreg |= ((_conf->timing.tMRD-1U) << FMC_SDTR1_TMRD_Pos)
				| ((_conf->timing.tXSR-1U) << FMC_SDTR1_TXSR_Pos)
				| ((_conf->timing.tRAS-1U) << FMC_SDTR1_TRAS_Pos)
				| ((_conf->timing.tDPL-1U) << FMC_SDTR1_TWR_Pos)
				| ((_conf->timing.tRCD-1U) << FMC_SDTR1_TRCD_Pos);

		WRITE_REG(SDRAM_INSTANCE ->SDTR[SDRAM_BANK2], tmpreg);
	}

	sdram_param_init();
}

err_t fmc_sdram_sendcommand(sdram_command_t cmd){
	err_t ret;

	__IO uint32_t tmpreg = READ_REG(SDRAM_INSTANCE->SDCMR);
	tmpreg &=~ (FMC_SDCMR_MODE | FMC_SDCMR_CTB2 | FMC_SDCMR_CTB1 | FMC_SDCMR_NRFS | FMC_SDCMR_MRD);

	tmpreg |= cmd.mode | cmd.targer | ((cmd.autorefresh_num-1U) << FMC_SDCMR_NRFS_Pos)
		   | (cmd.registermode << FMC_SDCMR_MRD_Pos);

	WRITE_REG(SDRAM_INSTANCE->SDCMR, tmpreg);

	ret = wait_bits(&(SDRAM_INSTANCE->SDSR), FMC_SDSR_BUSY, BITS_RESET, FMC_SDRAM_TIMEOUT);

	return ret;
}

void fmc_sdram_setrefreshrate(uint32_t refreshrate){
	__IO uint32_t tmpreg = READ_REG(SDRAM_INSTANCE->SDRTR);
	tmpreg &=~ FMC_SDRTR_COUNT;

	tmpreg |= (refreshrate << FMC_SDRTR_COUNT_Pos);

	WRITE_REG(SDRAM_INSTANCE->SDRTR, tmpreg);
}

static void sdram_param_init(void){
	sdram_command_t Command;

	Command.mode            = FMC_SDRAM_CMD_CLK_ENABLE;
	if(_conf->bank == SDRAM_BANK1)
		Command.targer      = FMC_SDRAM_CMD_TARGET_BANK1;
	else
		Command.targer      = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.autorefresh_num = 1;
	Command.registermode    = 0;
	fmc_sdram_sendcommand(Command);
	delay_ms(1);
	Command.mode            = FMC_SDRAM_CMD_PALL;
	fmc_sdram_sendcommand(Command);
	Command.mode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.autorefresh_num      = 2;
	fmc_sdram_sendcommand(Command);
	Command.mode            = FMC_SDRAM_CMD_LOAD_MODE;
	Command.registermode =  (uint32_t)0 | 0<<3 | 2<<4 | 0<<7 | 1<<9;
	fmc_sdram_sendcommand(Command);
	/* COUNT = [(SDRAM self refresh time / number of row) x  SDRAM CLK] – 20
		  = [(64ms/4096) * 100MHz] - 20 = 1562.5 - 20 ~ 1542 */
	fmc_sdram_setrefreshrate(_conf->refreshrate);
}


/**
 * @fn void fmc_sdram_hardware_init(void)
 * @brief
 *
 * @pre
 * @post
 */
static void fmc_sdram_hardware_init(void){
	gpio_port_clock_enable(GPIOC);
	gpio_port_clock_enable(GPIOD);
	gpio_port_clock_enable(GPIOE);
	gpio_port_clock_enable(GPIOF);
	gpio_port_clock_enable(GPIOG);


/**
 * Initialize Address pin.
 */
	gpio_pin_t pin;

	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A0_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A1_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A2_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A3_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A4_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A5_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A6_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A7_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A8_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A9_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A10_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_A11_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);

/**
 * Initialize dataI/O pin.
 */
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D0_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D1_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D2_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D3_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D4_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D5_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D6_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D7_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D8_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D9_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D10_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D11_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D12_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D13_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D14_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_D15_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);

/**
 * Initialize control pin.
 */
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_NBL0_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_NBL1_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);

	gpio_str_decode((char *)CONFIG_FMC_SDRAM_BA0_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_BA1_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);

	gpio_str_decode((char *)CONFIG_FMC_SDRAM_CLK_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_CAS_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_RAS_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_CKE_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_NE_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);
	gpio_str_decode((char *)CONFIG_FMC_SDRAM_NWE_PIN, &pin);
	gpio_set_function(pin.port, pin.pinnum, AF12_FMC_FSMC_SDIO_USB);

}


#endif /* PERIPHERAL_FMC_SDRAM_AVAILABLE */



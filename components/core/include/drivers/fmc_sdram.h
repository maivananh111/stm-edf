/*
 * fmc_sdram.h
 *
 *  Created on: Mar 22, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_DRV_FMC_SDRAM_H_
#define PERIPHERALS_DRV_FMC_SDRAM_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_FMC_SDRAM_EN && defined(FMC_Bank5_6)
#define PERIPHERAL_FMC_SDRAM_AVAILABLE 1

#include "common/error_check.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	SDRAM_BANK1 = 0U,
	SDRAM_BANK2 = 1U,
} sdram_bank_t;

typedef enum{
	SDRAM_COLUMN_SIZE_8BITS = 0U,
	SDRAM_COLUMN_SIZE_9BITS = FMC_SDCR1_NC_0,
	SDRAM_COLUMN_SIZE_10BITS = FMC_SDCR1_NC_1,
	SDRAM_COLUMN_SIZE_11BITS = (FMC_SDCR1_NC_0 | FMC_SDCR1_NC_1),
} sdram_coladdr_size_t;
typedef enum{
	SDRAM_ROW_SIZE_11BITS = 0U,
	SDRAM_ROW_SIZE_12BITS = FMC_SDCR1_NR_0,
	SDRAM_ROW_SIZE_13BITS = FMC_SDCR1_NR_1,
} sdram_rowaddr_size_t;
typedef enum{
	SDRAM_DATA_SIZE_8BITS  = 0U,
	SDRAM_DATA_SIZE_16BITS = FMC_SDCR1_MWID_0,
	SDRAM_DATA_SIZE_32BITS = FMC_SDCR1_MWID_1,
} sdram_data_size_t;

typedef enum{
	SDRAM_NUMBANK2 = 0,
	SDRAM_NUMBANK4 = FMC_SDCR1_NB,
} sdram_numbank_t;

typedef enum{
	SDRAM_LATENCY1 = FMC_SDCR1_CAS_0,
	SDRAM_LATENCY2 = FMC_SDCR1_CAS_1,
	SDRAM_LATENCY3 = (FMC_SDCR1_CAS_0 | FMC_SDCR1_CAS_1),
} sdram_latency_t;

typedef enum {
	SDRAM_CLOCK_DISABLE = 0U,
	SDRAM_CLOCK_2HCLK   = FMC_SDCR1_SDCLK_1,
	SDRAM_CLOCK_3HCLK   = (FMC_SDCR1_SDCLK_0 | FMC_SDCR1_SDCLK_1),
} sdram_clockcycle_t;

typedef enum{
	SDRAM_READNODELAY = 0U,
	SDRAM_READDELAY1 = FMC_SDCR1_RPIPE_0,
	SDRAM_READDELAY2 = FMC_SDCR1_RPIPE_1,
} sdram_readdelay_t;

typedef struct {
	sdram_bank_t 		 bank             = SDRAM_BANK1;
	uint32_t 			 frequency  	  = 100000000UL;
	sdram_clockcycle_t 	 clock_cycle      = SDRAM_CLOCK_2HCLK;
	sdram_coladdr_size_t coladdr_size     = SDRAM_COLUMN_SIZE_9BITS;
	sdram_rowaddr_size_t rowaddr_size     = SDRAM_ROW_SIZE_12BITS;
	sdram_data_size_t    data_size        = SDRAM_DATA_SIZE_16BITS;
	sdram_numbank_t      num_banks        = SDRAM_NUMBANK4;
	sdram_latency_t      cas_latency      = SDRAM_LATENCY2;
	sdram_readdelay_t 	 read_delay       = SDRAM_READDELAY2;
	bool write_protection                 = false;
	bool read_burst                       = true;
	uint32_t refreshrate                  = 1542U;
	struct{
		uint32_t tMRD = 2U; // Load mode register to active delay, MRS to new command.
		uint32_t tXSR = 1U; // Exit Self-Refresh delay, Self-Refresh exit time.
		uint32_t tRAS = 7U; // Self-Refresh time.
		uint32_t tRC  = 7U; // Common row cycle delay, RAS cycle time.
		uint32_t tDPL = 3U; // Write recovery time, data-in to precharge command time. WriteRecoveryTime >= SelfRefreshTime - RowToColumnDelay,
		uint32_t tRP  = 2U; // Common row precharge delay, RAS precharge time.
		uint32_t tRCD = 2U; // Row to column delay, RAS to CAS delay.
	} timing;
} sdram_config_t;

typedef struct{
	uint32_t mode;
	uint32_t targer;
	uint32_t autorefresh_num;
	uint32_t registermode;
} sdram_command_t;


void fmc_sdram_init(sdram_config_t *conf);

err_t fmc_sdram_sendcommand(sdram_command_t cmd);
void fmc_sdram_setrefreshrate(uint32_t refreshrate);



#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_FMC_SDRAM_AVAILABLE 0
#endif /* CONFIG_FMC_SDRAM_EN && defined(FMC_Bank5_6) */

#endif /* PERIPHERALS_FMC_SDRAM_H_ */

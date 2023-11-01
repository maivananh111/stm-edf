/*
 * embedflash.h
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_EMBEDFLASH_H_
#define PERIPHERALS_EMBEDFLASH_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_EMBEDFLASH_EN && defined(FLASH)
#define PERIPHERAL_EMBEDFLASH_AVAILABLE 1

#include "stdio.h"
#include "common/error_check.h"

#ifdef __cplusplus
extern "C"{
#endif


void embedflash_init(void);


uint32_t embedflash_calculate_latency(uint32_t freq);
void embedflash_set_latency(uint32_t latency);
void embedflash_update_latency(void);
uint32_t embedflash_get_latency(void);


err_t embedflash_unlock(void);
err_t embedflash_lock(void);
err_t embedflash_optionbytes_unlock(void);
err_t embedflash_optionbytes_lock(void);
err_t embedflash_optionbytes_launch(void);

uint32_t embedflash_get_page(uint32_t address_in_page);
uint32_t embedflash_get_sector(uint32_t address_in_sector);

err_t embedflash_erase(uint32_t erase_address);
err_t embedflash_writes(uint32_t writes_address, uint32_t *pdata);
err_t embedflash_reads(uint32_t reads_address, uint32_t *pdata);



#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_EMBEDFLASH_AVAILABLE 0
#endif /* CONFIG_PERIPH_EMBEDFLASH_EN && defined(FLASH) */

#endif /* PERIPHERALS_EMBEDFLASH_H_ */

/*
 * bits_check.h
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */

#ifndef BITS_CHECK_H_
#define BITS_CHECK_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "common/error_check.h"

#ifdef __cplusplus
extern "C"{
#endif



typedef enum{
	BITS_RESET = 0,
	BITS_SET,
} bits_lv_t;

enum {
	NO_WAIT = 0,
	DEFAULT_TIMEOUT = 1000U,
	WAIT_FOREVER = UINT32_MAX,
};


err_t check_bits(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t check_level);
__IO uint32_t get_bits(__IO uint32_t *register_value, uint32_t bits_mask);

err_t wait_bits(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t wait_level, uint32_t timeout);

err_t checkbits_while_waitbits(__IO uint32_t *register_check, uint32_t bits_check_mask, bits_lv_t check_level,
									  __IO uint32_t *register_wait, uint32_t bits_wait_mask, bits_lv_t wait_level, uint32_t timeout);

err_t wait_bits_fromISR(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t wait_level, uint32_t timeout);

err_t checkbits_while_waitbits_fromISR(__IO uint32_t *register_check, uint32_t bits_check_mask, bits_lv_t check_level,
									  __IO uint32_t *register_wait, uint32_t bits_wait_mask, bits_lv_t wait_level, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* BITS_CHECK_H_ */

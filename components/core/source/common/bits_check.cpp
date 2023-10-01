/*
 * bít_check.cpp
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */

#include "common/bits_check.h"
#include "drivers/system.h"
#include "drivers/rcc.h"

#define CLOCK_PER_MS 16000U
err_t check_bits(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t check_level){
	err_t err;

	if((check_level == BITS_SET)? ((*register_value & bits_mask) != 0U) : ((*register_value & bits_mask) == 0U)) err.id = E_PASS;
	else err.id = E_FAIL;

	return err;
}

__IO uint32_t get_bits(__IO uint32_t *register_value, uint32_t bits_mask){
	return (*register_value & bits_mask);
}


err_t wait_bits(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t wait_level, uint32_t timeout){
	err_t err;
	__IO uint32_t time_start = get_tick();

	while((wait_level == BITS_RESET)?(*register_value & bits_mask) : (!(*register_value & bits_mask))){
		if(timeout == WAIT_FOREVER){
			/** Wait forever, nothing to do */
		}
		else{
			if(get_tick() - time_start >= timeout) {
				err.id = E_TIMEOUT;
				break;
			}
		}
	}

	return err;
}


err_t checkbits_while_waitbits(__IO uint32_t *register_check, uint32_t bits_check_mask, bits_lv_t check_level,
									  __IO uint32_t *register_wait, uint32_t bits_wait_mask, bits_lv_t wait_level, uint32_t timeout){
	err_t err;
	__IO uint32_t time_start = get_tick();

	while((wait_level == BITS_RESET)? (*register_wait & bits_wait_mask) : (!(*register_wait & bits_wait_mask))){
		if((check_level == BITS_RESET)? (!(*register_check & bits_check_mask)) : (*register_check & bits_check_mask)){
			err.id = E_FAIL;
			break;
		}
		if(get_tick() - time_start >= timeout){
			err.id = E_TIMEOUT;
			break;
		}

	}

	return err;
}


err_t wait_bits_fromISR(__IO uint32_t *register_value, uint32_t bits_mask, bits_lv_t wait_level, uint32_t timeout){
	err_t err;
	__IO uint32_t i = 0;
	__IO uint32_t tickout = timeout * (rcc_get_bus_frequency(BUS_HCLK) / CLOCK_PER_MS);

	while(((timeout != WAIT_FOREVER)?(i++ < tickout) : 1) && ((wait_level == BITS_RESET)?(*register_value & bits_mask) : (!(*register_value & bits_mask)))){}
	if(i >= tickout) err.id = E_TIMEOUT;

	return err;
}


err_t checkbits_while_waitbits_fromISR(__IO uint32_t *register_check, uint32_t bits_check_mask, bits_lv_t check_level,
									  __IO uint32_t *register_wait, uint32_t bits_wait_mask, bits_lv_t wait_level, uint32_t timeout){
	err_t err;
	__IO uint32_t i = 0;
	__IO uint32_t tickout = timeout * (rcc_get_bus_frequency(BUS_HCLK) / CLOCK_PER_MS);

	while(((timeout != WAIT_FOREVER)?(i++ < tickout) : 1) && ((wait_level == BITS_RESET)?(*register_wait & bits_wait_mask) : (!(*register_wait & bits_wait_mask)))){
		if((check_level == BITS_RESET)? (!(*register_check & bits_check_mask)) : (*register_check & bits_check_mask)){
			err.id = E_FAIL;
			return err;
		}
	}
	if(i >= tickout) err.id = E_TIMEOUT;

	return err;
}



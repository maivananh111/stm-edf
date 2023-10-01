/*
 * rng.h
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_RNG_H_
#define PERIPHERALS_RNG_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_RNG_ENABLE && defined(RNG)
#define PERIPHERAL_RNG_AVAILABLE 1

#include "stdio.h"
#include "common/error_check.h"

#ifdef __cplusplus
extern "C" {
#endif


void rng_init(void);

uint32_t rng_get_random(void);
uint32_t rng_get_random_invert(void);
void rng_set_seed(uint32_t seed);
uint32_t rng_get_random_seed(void);


#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_RNG_AVAILABLE 0
#endif /* CONFIG_PERIPH_RNG_ENABLE && defined(RNG) */

#endif /* PERIPHERALS_RNG_H_ */

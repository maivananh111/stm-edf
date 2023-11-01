/*
 * trng.h
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_TRNG_H_
#define PERIPHERALS_TRNG_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if CONFIG_PERIPH_TRNG_EN && defined(RNG)
#define PERIPHERAL_TRNG_AVAILABLE 1

#include "stdio.h"
#include "common/error_check.h"

#ifdef __cplusplus
extern "C" {
#endif


void trng_init(void);

uint32_t trng_get_random(void);
uint32_t trng_get_random_invert(void);
void trng_set_seed(uint32_t seed);
uint32_t trng_get_random_seed(void);


#ifdef __cplusplus
}
#endif

#else
#define PERIPHERAL_TRNG_AVAILABLE 0
#endif /* CONFIG_PERIPH_TRNG_EN && defined(RNG) */

#endif /* PERIPHERALS_TRNG_H_ */

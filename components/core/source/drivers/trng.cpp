/*
 * trng.cpp
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */
#include "drivers/trng.h"
#if PERIPHERAL_TRNG_AVAILABLE

#include "drivers/system.h"
#include "drivers/sysinfo.h"


#define TRNG_TIMEOUT CONFIG_PERIPH_TRNG_DEFAULT_OPERATION_TIMEOUT


static uint32_t _seed = 0;
void trng_init(void){
	RCC -> AHB2ENR |= RCC_AHB2ENR_RNGEN;

	RNG -> CR |=  RNG_CR_RNGEN;

	trng_set_seed(dev_get_free_heap_size());
}

uint32_t trng_get_random(void){
	__IO uint32_t tick = get_tick();
	__IO uint32_t random_number = 0U;

	while(!(RNG -> SR & RNG_SR_DRDY)){
		if(get_tick() - tick > TRNG_TIMEOUT){
			break;
		}
	}
	random_number = RNG -> DR;

	return random_number;
}

uint32_t trng_get_random_invert(void){
	return ~trng_get_random();
}

void trng_set_seed(uint32_t seed){
	_seed = seed;
}

uint32_t trng_get_random_seed(void){
	__IO uint32_t rand = _seed;
	for(int i=0; i<2; i++){
		rand ^= trng_get_random();
		rand ^= trng_get_random_invert();
	}
	return rand;
}


#endif /* PERIPHERAL_TRNG_AVAILABLE */



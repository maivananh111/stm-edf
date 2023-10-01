/*
 * rng.cpp
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */
#include "drivers/rng.h"
#if PERIPHERAL_RNG_AVAILABLE

#include "drivers/system.h"
#include "drivers/sysinfo.h"


#define RNG_TIMEOUT 2U


static uint32_t _seed = 0;
void rng_init(void){
	RCC -> AHB2ENR |= RCC_AHB2ENR_RNGEN;

	RNG -> CR |=  RNG_CR_RNGEN;

	rng_set_seed(dev_get_free_heap_size());
}

uint32_t rng_get_random(void){
	__IO uint32_t tick = get_tick();
	__IO uint32_t random_number = 0U;

	while(!(RNG -> SR & RNG_SR_DRDY)){
		if(get_tick() - tick > RNG_TIMEOUT){
			break;
		}
	}
	random_number = RNG -> DR;

	return random_number;
}

uint32_t rng_get_random_invert(void){
	return ~rng_get_random();
}

void rng_set_seed(uint32_t seed){
	_seed = seed;
}

uint32_t rng_get_random_seed(void){
	__IO uint32_t rand = _seed;
	for(int i=0; i<2; i++){
		rand ^= rng_get_random();
		rand ^= rng_get_random_invert();
	}
	return rand;
}


#endif /* PERIPHERAL_RNG_AVAILABLE */



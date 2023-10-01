/*
 * heap_caps.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_SYSINFO_H_
#define PERIPHERALS_SYSINFO_H_

#include "devconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdio.h"

#ifdef __cplusplus
extern "C"{
#endif


void *heap_caps_malloc(size_t size);
void heap_caps_free(void *ptr);



#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_SYSINFO_H_ */

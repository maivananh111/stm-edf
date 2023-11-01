/*
 * lan87xx.h
 *
 *  Created on: Nov 1, 2023
 *      Author: anh
 */

#ifndef DRVPERDEV_LAN87XX_H_
#define DRVPERDEV_LAN87XX_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#if defined(ETH)

#include "stdio.h"
#include "common/error_check.h"
#include "common/bits_check.h"

typedef err_t (*lan87xx_initialize_f)(void);
typedef err_t (*lan87xx_deinitialize_f)(void);
typedef err_t (*lan87xx_readreg_f)(uint32_t, uint32_t, uint32_t *);
typedef err_t (*lan87xx_writereg_f)(uint32_t, uint32_t, uint32_t);
typedef err_t  (*lan87xx_gettick_f)(void);

#endif

#endif /* DRVPERDEV_LAN87XX_H_ */

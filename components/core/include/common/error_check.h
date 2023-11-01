/*
 * error_check.h
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "common/macro.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	E_PASS          = 0x0000,   /*!< Operation pass */
	E_FAIL          = 0x0001,   /*!< Operation failure */
	E_BUSY          = 0x0002,   /*!< Operation busy */
	E_TIMEOUT       = 0x0004,   /*!< Operation timed out */
	E_MEM           = 0x0008,   /*!< Out of memory */
	E_INVALID       = 0x0010,   /*!< Invalid argument */
	E_NULL          = 0x0020,   /*!< NULL argument */
	E_NOT_ALLOWED   = 0x0040,   /*!< Operation not allowed */
	E_NOT_FOUND     = 0x0080,   /*!< Requested resource not found */
	E_NOT_SUPPORTED = 0x0100,   /*!< Operation or feature not supported */
} errid_t;

typedef struct {
	errid_t id = E_PASS;
	int   line = 0U;
} err_t;


#define ERROR_CAPTURE(__err__) {\
	__err__.line = CODE_LINE;\
}
#define ERROR_SET(__err__, __id__) {\
	__err__.line = CODE_LINE;\
	__err__.id = __id__;\
}

#define IS_ERROR(__err__)			if(__err__.id != E_PASS && ((__err__.line = CODE_LINE) != 0U))
#define IS_PASS(__err__)            if(__err__.id == E_PASS && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_FAIL(__err__)          if(__err__.id == E_FAIL && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_BUSY(__err__)          if(__err__.id == E_BUSY && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_TIMEOUT(__err__)       if(__err__.id == E_TIMEOUT && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_MEM(__err__)           if(__err__.id == E_MEM && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_INVALID(__err__)       if(__err__.id == E_INVALID && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_NULL(__err__)          if(__err__.id == E_NULL && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_NOT_ALLOWED(__err__)   if(__err__.id == E_NOT_ALLOWED && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_NOT_FOUND(__err__)     if(__err__.id == E_NOT_FOUND && ((__err__.line = CODE_LINE) != 0U))
#define IS_E_NOT_SUPPORTED(__err__) if(__err__.id == E_NOT_SUPPORTED && ((__err__.line = CODE_LINE) != 0U))


#ifdef __cplusplus
}
#endif

#endif /* STATUS_H_ */

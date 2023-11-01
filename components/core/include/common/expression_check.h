/*
 * expression_check.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#ifndef EXPRESSION_CHECK_H_
#define EXPRESSION_CHECK_H_

#include "kconfig.h"
#include CONFIG_CMSIS_HEADER_FILE
#include "common/log_monitor.h"

#ifdef __cplusplus
extern "C"{
#endif


#define ASSERT_THEN_RETURN(__EXPRESSION__, __DO__, __LOG__, __TAG__, __FORMAT__, ...) {\
		if(__EXPRESSION__){\
			__DO__;\
			__LOG__(__TAG__, "%s[%d]>>> " __FORMAT__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			return;\
		}\
}

#define ASSERT_THEN_RETURN_VALUE(__EXPRESSION__, __DO__, __RETVAL__, __LOG__, __TAG__, __FORMAT__, ...) {\
		if(__EXPRESSION__){\
			__DO__;\
			__LOG__(__TAG__, "%s[%d]>>> " __FORMAT__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			return __RETVAL__;\
		}\
}


#define ASSERT_THEN_GOTO_LABEL(__EXPRESSION__, __DO__, __LABEL__, __LOG__, __TAG__, __FORMAT__, ...) {\
		if(__EXPRESSION__){\
			__DO__;\
			__LOG__(__TAG__, "%s[%d]>>> " __FORMAT__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			goto __LABEL__;\
		}\
}


#define ASSERT_THEN_BREAK(__EXPRESSION__, __DO__, __LOG__, __TAG__, __FORMAT__, ...) {\
		if(__EXPRESSION__){\
			__DO__;\
			__LOG__(__TAG__, "%s[%d]>>> " __FORMAT__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			break;\
		}\
}

#define ASSERT_THEN_CONTINUE(__EXPRESSION__, __DO__, __LOG__, __TAG__, __FORMAT__, ...) {\
		if(__EXPRESSION__){\
			__DO__;\
			__LOG__(__TAG__, "%s[%d]>>> " __FORMAT__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			continue;\
		}\
}


#ifdef __cplusplus
}
#endif

#endif /* EXPRESSION_CHECK_H_ */

/*
 * macro.h
 *
 *  Created on: Aug 5, 2023
 *      Author: anh
 */

#ifndef MACRO_H_
#define MACRO_H_

#ifdef __cplusplus
extern "C"{
#endif


#define bytes_to_words(bytes)                      		(bytes/4U)
#define bytes_to_kilobytes(bytes)                      	(bytes/1024U)
#define words_to_bytes(words)							(words*4U)
#define words_to_kilobytes(words)						((words*4U)/1024U)
#define kilobytes_to_bytes(kbytes)                      (kbytes*1024U)
#define kilobytes_to_words(kbytes)                      ((kbytes*1024U)/4U)

#define make_string(s) #s

#define CODE_FILE __FILE__
#define CODE_FUNC __FUNCTION__
#define CODE_LINE __LINE__
#define COMPILE_TIME __TIME__
#define COMPILE_DATE __DATE__

#define FUNC                std::function
#define FUNC_BIND           std::bind
#define FUNC_PARAM(x)       std::placeholders::_##x

#ifdef __cplusplus
}
#endif

#endif /* MACRO_H_ */

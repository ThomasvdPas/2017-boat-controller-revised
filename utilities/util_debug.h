/*
 * debug.h
 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "mbed.h"



//#define DEBUG 1
#define USE_TIMEOUT 1
//#define LOGGER 1

#ifdef DEBUG
	#define PRINT(...) pcMutex.lock();\
	pc.printf(__VA_ARGS__);\
	pc.printf("\r");\
	pcMutex.unlock();
#else
	#define PRINT(...)
#endif

#endif /* DEBUG_H_ */


/*
#ifdef LOGGER
	#define LOG_PRINT(...) log_print(__FILE__,__LINE__,__VA_ARGS__)
#else
	#define LOG_PRINT(...)
*/

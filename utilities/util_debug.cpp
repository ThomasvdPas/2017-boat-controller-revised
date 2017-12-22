/*
 * debug.cpp
 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#include "util_debug.h"
#include "rtos.h"
#include "mbed.h"

#ifdef DEBUG
Serial pc(USBTX, USBRX);
Mutex pcMutex;
#endif

/*
 * CanBuffer.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: boris
 */

#include <stdlib.h>
#include "mbed.h"
#include "CanBuffer.h"
#include "rtos.h"

extern Serial pc;
extern Mutex pcMutex;

CanBuffer::CanBuffer() : start(0), count(0) {}

CanBuffer::~CanBuffer() {}

bool CanBuffer::isFull() {
	return count == CANBUFFER_SIZE;
}

bool CanBuffer::isEmpty() {
	return count == 0;
}

void CanBuffer::insert(CANMessage m) {
	int end = (start + count) % CANBUFFER_SIZE;
	if(isFull()) {
		//free(buffer[end]);
		start = (start+1) % CANBUFFER_SIZE;
	} else {
		count++;
	}
	buffer[end] = m;
}

bool CanBuffer::read(CANMessage &m) {
	if(isEmpty())
		return false;
	m = buffer[start];
	start = (start+1) % CANBUFFER_SIZE;
	count--;
	return true;
}

/*
 * CanBuffer.h
 *
 *  Created on: Mar 4, 2014
 *      Author: boris
 */

#ifndef CANBUFFER_H_
#define CANBUFFER_H_

#include "mbed.h"

#define CANBUFFER_SIZE 64

class CanBuffer {
private:
	int start;
	int count;
	CANMessage buffer[CANBUFFER_SIZE];
public:
	CanBuffer();
	virtual ~CanBuffer();
	bool isFull();
	bool isEmpty();
	void insert(CANMessage m);
	bool read(CANMessage &m);
	void canInterrupt();
};

#endif /* CANBUFFER_H_ */

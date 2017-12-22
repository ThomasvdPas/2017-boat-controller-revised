//
// Created by jeroen on 20-5-17.
//

#ifndef CANMANAGER_H
#define CANMANAGER_H

#include "mbed.h"
#include "rtos.h"
#include "CanBuffer.h"
#include "config.h"

namespace sbt {

	class CanManager{
	private:
		CAN *canBus;
		Mutex *mutex;

	public:
		CanBuffer *canBusBuffer;

		CanManager(CAN *);
		WriteCanException writeCan(int, char*, char, int timeout_us = CAN_WRITE_DEFAULT_TIMEOUT_US);
		WriteCanException writeCanError(CanErrorCategory, uint8_t, int timeout_us = CAN_WRITE_DEFAULT_TIMEOUT_US );
		WriteCanException writeMaxonSpecificCanError(uint8_t, uint8_t, int timeout_us= CAN_WRITE_DEFAULT_TIMEOUT_US);
	};

}


#endif //CANMANAGER_H


#include "CanManager.h"

CanManager::CanManager(CAN *canBus) {
	this->canBus = canBus;
	this->canBusBuffer = new CanBuffer();
	this->mutex = new Mutex();
}

/**
 * Send message to the can bus
 */
WriteCanException CanManager::writeCan(int id, char* data, char len, int timeout_us) {
	Timer t;
	mutex->lock();
	t.start();
	while(!canBus->write(CANMessage(id, data, len))) {
		if(USE_TIMEOUT && t.read_us() >= timeout_us) {
			mutex->unlock();
			return WriteCanException::timeout;
		}
	}
	mutex->unlock();
	return WriteCanException::noException;
}

/**
 * Will send an error over the can bus
 */
WriteCanException CanManager::writeCanError(CanErrorCategory category, uint8_t errorCode, int timeout_us) {
	char data[2] = {(char)category, errorCode};
	return writeCan(WING_CONTROL_ERROR_CAN_ID, data, 2, timeout_us);
}

/**
 * Will send a maxon specific error over the can bus
 */
WriteCanException CanManager::writeMaxonSpecificCanError(uint8_t errorCodeMSB, uint8_t errorCodeLSB, int timeout_us) {
	char data[4] = {(char) CanErrorCategory::maxonmotor1, (char) MaxonMotorException::maxonSpecificException, errorCodeLSB, errorCodeMSB};
	return writeCan(WING_CONTROL_ERROR_CAN_ID, data, 4, timeout_us);
}

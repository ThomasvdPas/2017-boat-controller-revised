/*
 * Maxon.cpp
 *
 *  Created on: Mar 11, 2014
 *      Author: boris & bart
 */

#include "mbed.h"
#include "utilities/utilities.h"
#include "MaxonMotor.h"
#include "config.h"

extern Serial pc;
extern Mutex pcMutex;

namespace sbt {

MaxonMotor::MaxonMotor(CanBuffer* buffer, CanManager* canManager, Flash* flash, uint16_t nodeID)
		: acceleration(0),velocity(0),maxFollowingError(0),currentStatusword(0),positionActualValue(0) {
	this->buffer = buffer;
	this->canManager = canManager;
	this->flash = flash;
	this->nodeID = nodeID;

	TSDO_ID = DEF_TSDO_ID+nodeID;
	RSDO_ID = DEF_RSDO_ID+nodeID;
	MAXON_TPDO1 = DEF_MAXON_TPDO1+nodeID;
	MAXON_TPDO2 = DEF_MAXON_TPDO2+nodeID;
	MAXON_RPDO1 = DEF_MAXON_RPDO1+nodeID;
	MAXON_HEARTBEAT = DEF_MAXON_HEARTBEAT+nodeID;
	MAXON_COB_ID_EMCY = DEF_MAXON_COB_ID_EMCY+nodeID;

}

void MaxonMotor::setCurrentStatusword(uint16_t currentStatusword) {
	this->currentStatusword = currentStatusword;
}

void MaxonMotor::setPositionActualValue(int32_t positionActualValue) {
	this->positionActualValue = positionActualValue;
}

int32_t MaxonMotor::getPositionActualValue() {
	return this->positionActualValue;
}

/**
 * Wait for a SDO response message with a specific index and optional sub-index
 */
MaxonMotorException MaxonMotor::waitForSdoResponse(uint8_t indexMsb, uint8_t indexLsb, uint8_t subIndex) {
	Timer timer;
	timer.start();
	while(true) {
		if(USE_TIMEOUT == 1 && timer.read_ms() >= MAXON_MOTOR_TIMEOUT)
			return MaxonMotorException::timeout;
		CANMessage m;
		if(buffer->read(m)) {
			if(
				m.id == RSDO_ID &&
				indexMsb == (uint8_t)m.data[2] &&
				indexLsb == (uint8_t)m.data[1] &&
				subIndex == (uint8_t)m.data[3]
			) break;
		}
	}
	return MaxonMotorException::noException;
}

bool MaxonMotor::masking(uint16_t expectedPattern, uint16_t mask){
	if (expectedPattern == (currentStatusword & mask))
		return true;
	else
		return false;
}

MaxonMotorException MaxonMotor::waitForStatusword(uint16_t expectedPattern, uint16_t mask, int timeout) {
	Timer timer;
	timer.start();
	while(!masking(expectedPattern, mask)) {
		//PRINT("current: %x, expected: %x, mask: %x, masked: %x  \n", currentStatusword, expectedPattern, mask, currentStatusword&mask);
		if(USE_TIMEOUT == 1 && timer.read_ms() >= timeout)
			return MaxonMotorException::timeout;
	}
	return MaxonMotorException::noException;
}

/**
 * Send an SDO message and wait for the acknowledge response
 */
MaxonMotorException MaxonMotor::sendSdo(char message[8]) {
	//can->write(CANMessage(TSDO_ID, message, 8));
	canManager->writeCan(TSDO_ID, message, 8);
	return waitForSdoResponse(message[2], message[1], message[3]);
}

/**
 * As network manager, send a start message to the maxon node
 */
MaxonMotorException MaxonMotor::nmtStartMaxon() {
	Timer timer;
	timer.start();

	CANMessage m;

	resetCommunication();

	char dataNmtPreOperationalMaxon[2] = {0x80, this->nodeID};
	// wait for first hearbeat (bootup) message
	while(m.id != MAXON_HEARTBEAT) {
		if(USE_TIMEOUT == 1 && timer.read_ms() >= MAXON_MOTOR_NMT_BOOTUP_TIMEOUT)
			return MaxonMotorException::timeout;
		//can->write(CANMessage(0x00, dataNmtPreOperationalMaxon, 2));
		canManager->writeCan(0x00, dataNmtPreOperationalMaxon, 2);
		wait(0.1);
		buffer->read(m);
	}

	// start node

	char dataNmtStartMaxon[2] = {0x01, this->nodeID};
	//can->write(CANMessage(0x00, dataNmtStartMaxon, 2));
	canManager->writeCan(0x00, dataNmtStartMaxon, 2);

	return waitForStatusword(STATUS_WORD_SWITCH_ON_DISABLED, STATUS_WORD_SWITCH_ON_DISABLED_MASK);
}

/**
 * Send out homing instructions
 */
MaxonMotorException MaxonMotor::maxonHoming() {
	// Select homing operation mode
	char dataModesOfOperation[8] = {0b00101111,0x60,0x60,0x00,0x06,0x00,0x00,0x00};
	if(sendSdo(dataModesOfOperation) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomingOperation;

	// Set current threshold to 2000mA
	char dataCurrentTresholdHomingMode[8] = {0b00101011,0x80,0x20,0x00, 0xD0,0x07,0x00,0x00};
	if(sendSdo(dataCurrentTresholdHomingMode) == MaxonMotorException::timeout)
		return MaxonMotorException::setCurrentTreshold;
//
	// Set Homing Speed switch search to 2000 rpm
	char dataHomingSpeed[8] = {0b00100011,0x99,0x60,0x01, 0xD0,0x07,0x00,0x00};
	if(sendSdo(dataHomingSpeed) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomingSpeed;
//
//		// Home Offset
	int32_to_4_bytes_t neutralWings;
	neutralWings.i = -flash->getNeutralAngle();
	char dataHomingOffset[8] = {0b00100011,0x7C,0x60,0x00, neutralWings.c[0],neutralWings.c[1],neutralWings.c[2],neutralWings.c[3]};
	if(sendSdo(dataHomingOffset) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomingOffset;

		// Home position
	char dataHomingPosition[8] = {0b00100011,0x81,0x20,0x00, 0x00, 0x00, 0x00, 0x00};
	if(sendSdo(dataHomingPosition) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomePosition;

	// set Homing Acceleration to 1000 rpm
	char dataHomingAcceleration[8] = {0b00100011,0x9A,0x60,0x00, 0xE8, 0x03, 0x00, 0x00};
	if(sendSdo(dataHomingAcceleration) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomingSpeed;

	 /*Select type of homing method
	 0x22 = Index negative / Positive speed
	 0xFC (-4) = Current Threshold with negative speed
	 0xFD (-3) = Current Threshold with positive speed*/
	char dataHomingMethod[8] = {0b00101111,0x98,0x60,0x00, 0xFD,0x00,0x00,0x00};
	if(sendSdo(dataHomingMethod) == MaxonMotorException::timeout)
		return MaxonMotorException::setHomingMethod;

	// Start homing
	char dataStartHoming[8] = {0b00101011,0x40,0x60,0x00,0x1F,0x00,0x00,0x00};
	if(sendSdo(dataStartHoming) == MaxonMotorException::timeout)
		return MaxonMotorException::setStartHoming;

	// Block until homing target was reached
	if(waitForStatusword(STATUS_WORD_HOME_REACHED, STATUS_WORD_HOME_REACHED_MASK, MAXON_MOTOR_HOME_REACHED_TIMEOUT) == MaxonMotorException::timeout)
		return MaxonMotorException::maskHomeReached;

	return MaxonMotorException::noException;
}

MaxonMotorException MaxonMotor::disableDevice() {
	char dataShutdown[8] = {0b00101011, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	if(sendSdo(dataShutdown) == MaxonMotorException::timeout)
		return MaxonMotorException::setShutDown;
	return MaxonMotorException::noException;
}

MaxonMotorException MaxonMotor::enableDevice() {
	// "Shut Down" control word
	char dataShutdown[8] = {0b00101011,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
	if(sendSdo(dataShutdown) == MaxonMotorException::timeout)
		return MaxonMotorException::setShutDown;

	if(waitForStatusword(STATUS_WORD_READY_TO_SWITCH_ON, STATUS_WORD_READY_TO_SWITCH_ON_MASK) == MaxonMotorException::timeout)
		return MaxonMotorException::maskReadyToSwitchOn;

	// "Switch on" (enable) control word
	char dataSwitchOn[8] = {0b00101011,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};
	if(sendSdo(dataSwitchOn) == MaxonMotorException::timeout)
		return MaxonMotorException::setSwitchOn;

	// Block until device is "enabled"
	if(waitForStatusword(STATUS_WORD_OPERATION_ENABLE, STATUS_WORD_OPERATION_ENABLE_MASK) == MaxonMotorException::timeout)
		return MaxonMotorException::maskOperationEnable;

	return MaxonMotorException::noException;
}

MaxonMotorException MaxonMotor::initProfilePositionMode() {
	// Select the profile position mode
	char dataModesOfOperation[8] = {0b00101111,0x60,0x60,0x00,0x01,0x00,0x00,0x00};
	if(sendSdo(dataModesOfOperation) == MaxonMotorException::timeout)
		return MaxonMotorException::setPositionProfileOperation;

	return MaxonMotorException::noException;
}


MaxonMotorException MaxonMotor::initPositionMode() {
	// Select the position mode
	char dataModesOfOperation[8] = {0b00101111,0x60,0x60,0x00,(char) -1,0x00,0x00,0x00};
	if(sendSdo(dataModesOfOperation) == MaxonMotorException::timeout)
		return MaxonMotorException::setPositionOperation;

	return MaxonMotorException::noException;
}

MaxonMotorException MaxonMotor::positionModeSettingValue(int32_t position) {
	// Set target position
	uint8_t *positionArray = reinterpret_cast<uint8_t*>(&position);
	char positionModeSettingValue[8] = {0b00100011,0x62,0x20,0x00,0,0,0,0};
	positionModeSettingValue[4] = positionArray[0];
	positionModeSettingValue[5] = positionArray[1];
	positionModeSettingValue[6] = positionArray[2];
	positionModeSettingValue[7] = positionArray[3];
	if(sendSdo(positionModeSettingValue) == MaxonMotorException::timeout)
		return MaxonMotorException::positionModeSettingValue;

	return MaxonMotorException::noException;

}


MaxonMotorException MaxonMotor::goToNeutral() {
	if (getPositionActualValue() != MAXON_NEUTRAL_POSITION){
		uint32_t target = MAXON_NEUTRAL_POSITION;
		uint8_t *arr = reinterpret_cast<uint8_t*>(&target);
		char dataTarget[6] = {arr[0], arr[1], arr[2], arr[3], 0x3F, 0x00};
		//can->write(CANMessage(MAXON_RPDO1, dataTarget, 6));
		canManager->writeCan(MAXON_RPDO1, dataTarget, 6);
		return MaxonMotorException::noException;
	} else {
		return MaxonMotorException::neutralReached;
	}

}


MaxonMotorException MaxonMotor::instructMaxon(int32_t feedback){
	if (abs(feedback)<QC_THRESHOLD){
		return MaxonMotorException::noException;
	}
	int32_t target = feedback;
	uint8_t *arr = reinterpret_cast<uint8_t*>(&target);
	char dataTarget[6] = {arr[0], arr[1], arr[2], arr[3], 0x3F, 0x00};
	//can->write(CANMessage(MAXON_RPDO1, dataTarget, 6));
	canManager->writeCan(MAXON_RPDO1, dataTarget, 6);

	return MaxonMotorException::noException;
}

void MaxonMotor::resetCommunication(){
	char dataNmtResetCommunication[2] = {0x82, this->nodeID};
	//can->write(CANMessage(0x00, dataNmtResetCommunication, 2));
	canManager->writeCan(0x00, dataNmtResetCommunication, 2);
}

MaxonMotorException MaxonMotor::resetEPOS2(){
	// reset fault
	char dataFaultReset[8] = {0b00101011,0x40,0x60,0x00,0x80,0x00,0x00,0x00};
	PRINT("STATUSWORD: %x\n", currentStatusword);
	if(sendSdo(dataFaultReset) == MaxonMotorException::timeout)
		return MaxonMotorException::setResetFault;
	return enableDevice();
}

MaxonMotorException MaxonMotor::resetFault(uint16_t errorCode) {
	if(errorCode == 0x8120 || errorCode == 0x8130) {
		char dataNmtResetNode[2] = {0x82, this->nodeID};
		//can->write(CANMessage(0x00, dataNmtResetNode, 2));
		canManager->writeCan(0x00, dataNmtResetNode, 2);
	} else {
		// wait for device to be in fault state
		if(waitForStatusword(STATUS_WORD_FAULT, STATUS_WORD_FAULT_MASK) == MaxonMotorException::timeout)
			return MaxonMotorException::maskFault;

		// reset fault
		char dataFaultReset[8] = {0b00101011,0x40,0x60,0x00,0x80,0x00,0x00,0x00};
		if(sendSdo(dataFaultReset) == MaxonMotorException::timeout)
			return MaxonMotorException::setResetFault;

		// re-enable device
		return enableDevice();
	}
	return MaxonMotorException::noException;
}

uint32_t MaxonMotor::getAcceleration() const {
	return acceleration;
}

MaxonMotorException MaxonMotor::setAcceleration(uint32_t acceleration) {
	this->acceleration = acceleration;

	uint8_t *arr = reinterpret_cast<uint8_t*>(&acceleration);

	char dataAcceleration[8] = {0b00100011, 0x83, 0x60, 0x00, arr[0], arr[1], arr[2], arr[3]};
	if(sendSdo(dataAcceleration) == MaxonMotorException::timeout)
		return MaxonMotorException::setAcceleration;

	char dataDeceleration[8] = {0b00100011, 0x84, 0x60, 0x00, arr[0], arr[1], arr[2], arr[3]};
	if(sendSdo(dataDeceleration) == MaxonMotorException::timeout)
		return MaxonMotorException::setDeceleration;

	return MaxonMotorException::noException;
}

uint32_t MaxonMotor::getMaxFollowingError() const {
	return maxFollowingError;
}

MaxonMotorException MaxonMotor::setMaxFollowingError(uint32_t maxFollowingError) {
	this->maxFollowingError = maxFollowingError;

	uint8_t *arr = reinterpret_cast<uint8_t*>(&maxFollowingError);
	char dataMaxFollowingError[8] = {0b00100011, 0x65, 0x60, 0x00, arr[0], arr[1], arr[2], arr[3]};
	if(sendSdo(dataMaxFollowingError) == MaxonMotorException::timeout)
		return MaxonMotorException::setMaxFollowingError;

	return MaxonMotorException::noException;
}

uint32_t MaxonMotor::getVelocity() const {
	return velocity;
}

MaxonMotorException MaxonMotor::setVelocity(uint32_t velocity) {
	this->velocity = velocity;

	uint8_t *arr = reinterpret_cast<uint8_t*>(&velocity);
	char dataVelocity[8] = {0b00100011, 0x81, 0x60, 0x00, arr[0], arr[1], arr[2], arr[3]};
	if(sendSdo(dataVelocity) == MaxonMotorException::timeout)
		return MaxonMotorException::setVelocity;

	return MaxonMotorException::noException;
}

float MaxonMotor::getAngleFront(){
	float angle = (float)(positionActualValue); // ANGLE_TO_QC_FRONT);
	return angle;
}

float MaxonMotor::getAngleRear(){
	float angle = (float)(positionActualValue); // ANGLE_TO_QC_REAR);
	return angle;
}

} /* namespace sbt */

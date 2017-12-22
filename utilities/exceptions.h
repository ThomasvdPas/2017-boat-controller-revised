/*
 * exceptions.h
 *
 *  Created on: Mar 25, 2014
 *      Author: boris
 */

#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

enum class GeneralException : uint8_t {
	watchdogTimerReset
};

enum class XsensException : uint8_t {
	noException=0,
	timeout=1,

	wrongMessageHeader=2,
	checksum=3,

	messageTooLarge=4,

	errorMessageReceived=5,

	xsensPeriodSentInvalidRange=6,
	xsensMessageSentInvalid=7,
	xsensTimerOverflow=8,
	xsensBaudRateInvalidRange=9,
	xsensParameterSentInvalid=10,

	messageBuffered=11
};

enum class HeightSensorException : uint8_t {
	noException=0,
	timeout=1,

	sensor=2,
	crc=3,
	wrongMessage=4,

	setFilters=5,
	setNumberOfSamplesToBeAveraged=6,
	setInterval=7,
	setTransmittedPulses=8
};

enum class MaxonMotorException : uint8_t {
	noException=0,
	timeout=1,

	setHomingOperation=2,
	setCurrentTreshold=3,
	setHomingSpeed=4,
	setHomingOffset=5,
	setHomePosition=6,
	setHomingMethod=7,
	setShutDown=8,
	setSwitchOn=9,
	setStartHoming=10,

	setPositionProfileOperation=11,
	setPositionOperation=12,
	setTarget=13,

	positionModeSettingValue=14,

	maskReadyToSwitchOn=15,
	maskOperationEnable=16,
	maskHomeReached=17,
	maskFault=18,

	setResetFault=19,

	maxonSpecificException=20,

	targetSetTooHigh=21,
	targetSetTooLow=22,

	setAcceleration=23,
	setDeceleration=24,
	setMaxFollowingError=25,
	setVelocity=26,

	neutralReached=27
};

enum class WriteCanException : uint8_t {
	noException,
	timeout
};


enum class CanErrorCategory : uint8_t {
	general,
	xsens,
	heightsensor1,
	heightsensor2,
	maxonmotor1,
	maxonmotor2
};


#endif /* EXCEPTIONS_H_ */

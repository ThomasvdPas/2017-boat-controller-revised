/*
 * Maxon.h
 *
 *  Created on: Mar 11, 2014
 *      Author: boris & Bart
 */

#ifndef MAXON_H_
#define MAXON_H_


#include "mbed.h"
#include "utilities/utilities.h"
#include "CanBuffer.h"
#include "Flash.h"
#include "CanManager.h"

// CANOpen definitions
#define MAX_ANGLE							8
#define DEF_TSDO_ID 							0x0600
#define DEF_RSDO_ID 							0x0580
#define DEF_MAXON_TPDO1 						0x0180
#define DEF_MAXON_TPDO2 						0x0280
#define DEF_MAXON_RPDO1 						0x0200
#define DEF_MAXON_HEARTBEAT						0x0700
#define DEF_MAXON_COB_ID_EMCY					0x0080

// Status words and their masks
#define STATUS_WORD_READY_TO_SWITCH_ON		0b0000000100100001
#define STATUS_WORD_READY_TO_SWITCH_ON_MASK	0b0100000101111111
#define STATUS_WORD_OPERATION_ENABLE 		0b0000000100110111
#define STATUS_WORD_OPERATION_ENABLE_MASK	0b0100000101111111
#define STATUS_WORD_HOME_REACHED			0b1001010000000000
#define STATUS_WORD_HOME_REACHED_MASK		0b1001010000000000
#define	 STATUS_WORD_SWITCH_ON_DISABLED		0b0000000100000000
#define	 STATUS_WORD_SWITCH_ON_DISABLED_MASK	0b0100000100001000
#define	 STATUS_WORD_FAULT					0b0000000100001000
#define	 STATUS_WORD_FAULT_MASK				0b0100000101111111

// Timeouts
#define MAXON_MOTOR_TIMEOUT 				500
#define MAXON_MOTOR_NMT_BOOTUP_TIMEOUT		5000
#define MAXON_MOTOR_HOME_REACHED_TIMEOUT 	40000

#define ANGLE_TO_QC_FRONT					8427
#define ANGLE_TO_QC_REAR					386355
#define QC_THRESHOLD						10
#define RANGE_FRONT							190000
#define SAFETYMIN_FRONT						5000
#define RANGE_REAR							0	//TODO: find range
#define SAFETYMIN_REAR						50000




namespace sbt {

class MaxonMotor {
private:
	uint32_t acceleration;
	uint32_t velocity;
	uint32_t maxFollowingError;
	uint16_t nodeID;

	CanBuffer* buffer;
	CanManager* canManager;
	Flash* flash;

	uint16_t currentStatusword;
	//int32_t positionActualValue;

public:
	uint16_t TSDO_ID;
	uint16_t RSDO_ID;
	uint16_t MAXON_TPDO1;
	uint16_t MAXON_TPDO2;
	uint16_t MAXON_RPDO1;
	uint16_t MAXON_HEARTBEAT;
	uint16_t MAXON_COB_ID_EMCY;

	int32_t positionActualValue;
	MaxonMotor(CanBuffer* buffer, CanManager* can, Flash* flash, uint16_t nodeID);
	void setCurrentStatusword(uint16_t currentStatusword);
	void setPositionActualValue(int32_t positionActualValue);
	int32_t getPositionActualValue();
	MaxonMotorException waitForSdoResponse(uint8_t indexMsb, uint8_t indexLsb, uint8_t subIndex = 0x00);
	MaxonMotorException waitForStatusword(uint16_t expectedPattern, uint16_t mask, int timeout=MAXON_MOTOR_TIMEOUT);
	MaxonMotorException sendSdo(char message[8]);
	MaxonMotorException nmtStartMaxon();
	MaxonMotorException disableDevice();
	MaxonMotorException enableDevice();
	MaxonMotorException maxonHoming();
	MaxonMotorException initProfilePositionMode();
	MaxonMotorException initPositionMode();
	MaxonMotorException positionModeSettingValue(int32_t position);
	void resetCommunication();
	MaxonMotorException resetEPOS2();
	MaxonMotorException resetFault(uint16_t errorCode);
	uint32_t getAcceleration() const;
	MaxonMotorException setAcceleration(uint32_t acceleration);
	uint32_t getMaxFollowingError() const;
	MaxonMotorException setMaxFollowingError(uint32_t maxFollowingError);
	uint32_t getVelocity() const;
	MaxonMotorException setVelocity(uint32_t velocity);
	bool masking(uint16_t expectedPattern, uint16_t mask);
	MaxonMotorException instructMaxon(int32_t feedback);
	MaxonMotorException goToNeutral();
	float getAngleFront();
	float getAngleRear();
};

} /* namespace sbt */

#endif /* MAXON_H_ */

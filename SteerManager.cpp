//
// Created by jeroen on 6-7-17.
//

#include "SteerManager.h"

SteerManager::SteerManager() {
	this->steerDataMutex = new Mutex();
	this->steerData = new SteerData();
}

float SteerManager::getReferenceHeight() {
	this->steerDataMutex->lock();
	float h_ref = steerData->height / 100.0f * 0.8f - 0.2f; // Max height is 60cm
	this->steerDataMutex->unlock();
	return h_ref;
}

void SteerManager::parseCanMessage(CANMessage *m) {
	steerDataMutex->lock();
	steerData->heightOn = 	m->data[0] & 0b00000001;
	steerData->backward = 	m->data[0] & 0b00000010;
	steerData->talk = 		m->data[0] & 0b00000100;
	steerData->screen = 		m->data[0] & 0b00001000;
	steerData->throttle = m->data[1];
	steerData->roll = ((float)m->data[2]) * CAN_HUNDRED_PERCENT_FACTOR;
	steerData->pitch = m->data[3];
	steerData->height = ((float)m->data[4]) * CAN_HUNDRED_PERCENT_FACTOR;
	steerData->steerAngle = m->data[5] * CAN_ANGLE_FACTOR;
	steerData->temperature = m->data[6];

	if (this->initializeSteerAngle) {
		this->previousAngle = steerData->steerAngle;
		this->initializeSteerAngle = false;
	} else {
		if (abs(this->previousAngle - steerData->steerAngle) > 180.0f) {
			// Jump greater than 180 degrees detected
			if (steerData->steerAngle > this->previousAngle) {
				this->steerAngleWrapped -= this->previousAngle + (360.0f - steerData->steerAngle);
			} else {
				this->steerAngleWrapped += 360.0f - this->previousAngle + steerData->steerAngle;
			}
			this->previousAngle = steerData->steerAngle;
		} else {
			this->steerAngleWrapped += steerData->steerAngle - this->previousAngle;
		}
	}
	steerDataMutex->unlock();

}

void SteerManager::tryCalibrateSteerAngle(Xsens *xsens) {
	xsens->xsensData_mutex->lock();
	if (xsens->xsensData.velocity_xy > 3.0f) {
		this->calib_avgRotz = this->calib_avgRotz * 0.9f + abs(xsens->xsensData.rot.z) * 0.1f;
		this->calib_angle = this->calib_angle * 0.9f + this->steerAngleWrapped * 0.1f;

		if (this->calib_avgRotz < 0.01f) {
			if (this->calibrationCount++ == 0) {
				this->steerAngleOffset = this->calib_angle;
			} else {
				this->steerAngleOffset = this->steerAngleOffset / this->calibrationCount * (this->calibrationCount - 1) + this->calib_angle / this->calibrationCount;
			}

			// Delay the next calibration
			this->calib_avgRotz = 1.0f;

		}
	} else {
		// Delay the next calibration
		this->calib_avgRotz = 0.1f;
	}
	xsens->xsensData_mutex->unlock();
}

float SteerManager::getSteerAngle() {
	return this->steerAngleWrapped - this->steerAngleOffset;
}

float SteerManager::getRudderDeflection() {
	return (this->steerAngleWrapped - this->steerAngleOffset)*STEER_TO_RUDDER_RATIO;
}

void SteerManager::CANReport(CanManager *canManager) {
	two_int32_to_8_bytes_t steerAngleMessage;
	steerAngleMessage.i[0] = (int32_t)(this->getSteerAngle());
	steerAngleMessage.i[1] = (int32_t)(-this->steerAngleOffset);
	int canId = REPORT_TO_CAN_STEERANGLE;
	canManager->writeCan(canId, steerAngleMessage.c, 8);

}
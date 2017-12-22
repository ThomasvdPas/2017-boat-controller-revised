//
// Created by jeroen on 6-7-17.
//

#ifndef STEERMANAGER_H
#define STEERMANAGER_H

#include "rtos.h"
#include "mbed.h"
#include "utilities/utilities.h"
#include "config.h"
#include "Xsens/Xsens.h"
#include "CanManager.h"


namespace sbt {

	class SteerManager {

	private:
		Mutex *steerDataMutex;
		uint16_t calibrationCount = 0;
		float steerAngleOffset = 0.0f;
		float steerAngleWrapped = 0.0f;
		float previousAngle = 0.0f;
		float calib_avgRotz = 0.0f;
		float calib_angle = 0.0f;
		bool initializeSteerAngle = true;

	public:
		SteerData *steerData;

		SteerManager();
		float getReferenceHeight();
		void parseCanMessage(CANMessage *);
		void tryCalibrateSteerAngle(Xsens *);
		float getSteerAngle();
		float getRudderDeflection();
		void CANReport(CanManager *);

	};


}

#endif //STEERMANAGER_H

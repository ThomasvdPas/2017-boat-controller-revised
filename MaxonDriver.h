//
// Created by jeroen on 20-5-17.
//

#ifndef MAXONDRIVER_H
#define MAXONDRIVER_H

#include "rtos.h"
#include "mbed.h"
#include "utilities/utilities.h"
#include "config.h"
#include "MaxonMotor.h"
#include "CanManager.h"

#define MAXON_STACK_SIZE		2048

namespace sbt {

	class MaxonDriver {
	private:
		CanBuffer *canMaxonBuffer;
		Flash *flash;
		uint16_t currentMaxonError;
		CanManager *canMaxon;
		uint16_t Status;
		CanManager *canManager;
		bool ready;


		void errorState(uint8_t);
		int32_t pitch = 0;
		float pitchRad = 0;
		bool heightOn;
		Thread *ownThread;

		void setPitch(int32_t);

	public:
		uint16_t nodeID;
		MaxonMotor *maxonMotor;
		float steerRoll;
		bool reachedBounds;
		uint16_t reachedBoundCount;

		MaxonDriver(CanManager *, CanManager *, uint16_t);
		void setOwnThread(Thread *);
		void CANReport(CanManager *);
		void setHeightOn(bool on);
		bool getHeightOn();
		void setup();
		void reset();
		void canIsr(CANMessage);
		void updateParameters();
		void maxonSetup();
		void thread (void const *);
		void setSteerRoll(float);
		void setPitchRad(float);
		bool isReady();
	};

}
#endif //MAXONDRIVER_H

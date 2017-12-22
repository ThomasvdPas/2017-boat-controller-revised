//
// Created by jeroen on 4-6-17.
//

#ifndef SENIXMANAGER_H
#define SENIXMANAGER_H

#include "mbed.h"
#include "rtos.h"
#include "config.h"
#include "utilities/utilities.h"
#include "HeightSensor.h"
#include "SteerManager.h"

namespace sbt {

	class SenixManager {

	private:
		uint32_t pulseCount = 0;
		uint32_t timerTickCount = 0;
		uint8_t wantPulse = 0;
		bool startup = true;


	public:
		Thread *senixThread1;
		Thread *senixThread2;
		HeightSensor *heightSensor1;
		HeightSensor *heightSensor2;
		SteerManager *steerManager;
		Thread *ownThread;
		Thread *processThread;

		void senix_tick_heightOff();
		void senix_tick_heightOn();
		SenixManager();
		void thread();

	};
}

#endif //SENIXMANAGER_H

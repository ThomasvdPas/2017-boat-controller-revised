
#include "SenixManager.h"

extern Serial pc;
extern Mutex pcMutex;


namespace sbt {

	SenixManager::SenixManager() {

	}

	void SenixManager::senix_tick_heightOff() {
		this->wantPulse = 5;
	}

	void SenixManager::senix_tick_heightOn () {
		this->timerTickCount++;
		if (this->steerManager->steerData->heightOn || this->wantPulse > 0) {

			this->wantPulse = max(0, this->wantPulse - 1);

			this->senixThread1->signal_set(0x4);
			this->senixThread2->signal_set(0x4);
			this->heightSensor1->requestHeight();
			this->heightSensor2->requestHeight();
//			this->ownThread->signal_set(0x5);
		}
	}

	void SenixManager::thread() {

	}

}
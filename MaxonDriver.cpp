//
// Created by jeroen on 20-5-17.
//

#include "MaxonDriver.h"

 extern Serial pc;
extern Mutex pcMutex;
extern "C" void mbed_reset();

MaxonDriver::MaxonDriver(CanManager *canMaxon, CanManager *canManager, uint16_t nodeID) {
	this->canMaxon = canMaxon;
	this->canManager = canManager;
	this->canMaxonBuffer = new CanBuffer();
	this->flash = new Flash(); // TODO gaat het goed als we meerdere instances hebben van flash?
	this->nodeID = nodeID;
	currentMaxonError = 0;
	this->reachedBoundCount = 0;
	this->heightOn = false;
	this->ready = false;
	this->maxonMotor = new MaxonMotor(this->canMaxonBuffer, this->canMaxon, this->flash, nodeID);
}

void MaxonDriver::setSteerRoll(float roll) {
	if (this->steerRoll != roll) {
		this->steerRoll = roll;
	}
	if (!this->heightOn) {
		this->ownThread->signal_set(0x3);
	}
}

void MaxonDriver::setPitch(int32_t pitch) {
	if (max(pitch, this->pitch)-min(pitch, this->pitch) > MAXON_NEXT_PITCH_MINIMAL_DISTANCE && this->isReady()) {
		this->pitch = pitch;
		this->ownThread->signal_set(0x3);
	} else if (max(pitch, this->pitch)-min(pitch, this->pitch) > MAXON_NEXT_PITCH_MINIMAL_DISTANCE && !this->isReady()) {
		this->pitch = pitch;
	}
}

void MaxonDriver::setPitchRad(float angle) {
	if (this->pitchRad == angle) {
		// There is no change
		return;
	}
	this->pitchRad = angle;
	float angleFromZero = angle - MAXON_MINRAD;
	float anglePercentage = angleFromZero / MAXON_TOTALRAD;
	int32_t position = (int32_t)(MAXON_MAX_NEGATIVE_ANGLE * (1-anglePercentage));

//	PRINT("pos: %d \t %f \t %f \t %f \t %f\n", position, anglePercentage, angle, angleFromZero, MAXON_TOTALRAD);
	reachedBounds = false;
	if (position > MAXON_MAX_POSITIVE_ANGLE - MAXON_POSITION_MARGIN) {
		position = MAXON_MAX_POSITIVE_ANGLE - MAXON_POSITION_MARGIN;
		reachedBounds = true;
		this->reachedBoundCount++;
	} else if (position < MAXON_MAX_NEGATIVE_ANGLE + MAXON_POSITION_MARGIN) {
		position = MAXON_MAX_NEGATIVE_ANGLE + MAXON_POSITION_MARGIN;
		reachedBounds = true;
		this->reachedBoundCount++;
	};

	// Be sure to never listen to extreme ddiff
	if (!reachedBounds) {
		this->setPitch(position);
	}
}


void MaxonDriver::setHeightOn(bool on) {
	if (this->heightOn && !on && this->isReady()) {
		this->heightOn = false;
		this->ownThread->signal_set(0x3);
	} else {
		this->heightOn = on;
	}
}


void MaxonDriver::setup() {
	PRINT("Maxon setup\n");
	maxonMotor->resetCommunication();
	maxonMotor->disableDevice();

	MaxonMotorException nmtException = maxonMotor->nmtStartMaxon();
	if(nmtException != MaxonMotorException::noException) {
		PRINT("NMT ERROR, %i \n", nmtException);
		setBit(&Status, StatusMaxon);
		//reportToCan(); //TODO trigger a report to can thread
		errorState((uint8_t) nmtException);
	}

	MaxonMotorException enableDeviceException = maxonMotor->enableDevice();
	if(enableDeviceException != MaxonMotorException::noException) {
		PRINT("ENABLE ERROR\n");
		setBit(&Status, StatusMaxon);
		//reportToCan(); //TODO trigger a report to can thread
		errorState((uint8_t) enableDeviceException);
	}

	updateParameters();
	
	MaxonMotorException homingException = maxonMotor->maxonHoming();
	if(homingException != MaxonMotorException::noException) {
		PRINT("HOMING ERROR exception: %i\n", homingException);
		setBit(&Status, StatusMaxon);
		//reportToCan(); //TODO trigger a report to can thread
		errorState( (uint8_t) homingException);
	}
	MaxonMotorException initPositionException = maxonMotor->initProfilePositionMode();
	if(initPositionException != MaxonMotorException::noException) {
		PRINT("INIT ERROR\n");
		setBit(&Status, StatusMaxon);
		//reportToCan(); //TODO trigger a report to can thread
		errorState( (uint8_t) initPositionException);
	}

	setBit(&Status, StatusHoming);
	return;

}

void MaxonDriver::CANReport(CanManager *canManager) {
	if (this->isReady()) {
		two_int16_to_4_bytes_t maxonMessage;
		maxonMessage.i[1] = (int16_t)(-this->pitch / 1000);
		maxonMessage.i[0] = (int16_t)(-this->maxonMotor->getPositionActualValue() / 1000);
		int canId = REPORT_TO_CAN_MAXON + this->nodeID - 1;
		canManager->writeCan(canId, maxonMessage.c, 4);
	}
}

bool MaxonDriver::isReady() {
	return ready;
}

void MaxonDriver::errorState(uint8_t exception){
	int cnt = 10;
	while(cnt > 0){
		canManager->writeCanError(CanErrorCategory::maxonmotor1, exception);//TODO also provide nodeID

		wait_ms(100);
		cnt--;
	}
	mbed_reset();

}


void MaxonDriver::updateParameters() {
	if(maxonMotor->getAcceleration() != flash->getActuatorAcceleration()) {
		MaxonMotorException e = maxonMotor->setAcceleration(flash->getActuatorAcceleration());
		if(e != MaxonMotorException::noException) {
			canManager->writeCanError(CanErrorCategory::maxonmotor1, (uint8_t) e); //TODO also provide nodeID
		}
	}
	if(maxonMotor->getVelocity() != flash->getActuatorVelocity()) {
		MaxonMotorException e = maxonMotor->setVelocity(flash->getActuatorVelocity());
		if(e != MaxonMotorException::noException) {
			canManager->writeCanError(CanErrorCategory::maxonmotor1, (uint8_t) e); //TODO also provide nodeID
		}
	}
	if(maxonMotor->getMaxFollowingError() != flash->getActuatorMaxFollowingError()) {
		MaxonMotorException e = maxonMotor->setMaxFollowingError(flash->getActuatorMaxFollowingError());
		if(e != MaxonMotorException::noException) {
			canManager->writeCanError(CanErrorCategory::maxonmotor1, (uint8_t) e); //TODO also provide nodeID
		}
	}
}

void MaxonDriver::canIsr(CANMessage m) {

	if (m.id == maxonMotor->MAXON_COB_ID_EMCY) {
		currentMaxonError = toUint16(m.data[0], m.data[1]);
	} else if (m.id == maxonMotor->MAXON_TPDO1) {
		maxonMotor->setCurrentStatusword(toUint16(m.data[1], m.data[0]));
	} else if (m.id == maxonMotor->MAXON_TPDO2) {
		int32_to_4_bytes_t actualValue;
		memcpy(actualValue.c, m.data, 4);
		maxonMotor->setPositionActualValue(actualValue.i);
	} else {
		canMaxonBuffer->insert(m);
	}
}

void MaxonDriver::reset() {
	maxonMotor->resetEPOS2();
}

void MaxonDriver::setOwnThread(Thread *thread) {
	this->ownThread = thread;
}

void MaxonDriver::thread (void const *) {

	PRINT("Maxon thread started.\n");

	// Start the motor and begin homing
	this->setup();

	//TODO make this speed configurable
	maxonMotor->setVelocity(7000);
	maxonMotor->setAcceleration(100000);

	PRINT("Maxon setup done.\n");

	maxonMotor->enableDevice();
	maxonMotor->initProfilePositionMode();

	ready = true;

	//TODO get Controller gains
	//getGains(&gains);

	while (1) {

		// reset maxon if needed
		if (currentMaxonError) {
			uint16_to_2_bytes_t maxonErr;
			maxonErr.i = currentMaxonError;
			canManager->writeMaxonSpecificCanError(maxonErr.c[0], maxonErr.c[1]);
		}

		maxonMotor->instructMaxon(pitch);
		int32_to_4_bytes_t controlCan;
		controlCan.i = pitch;
		canManager->writeCan(REPORT_TO_CAN_CONTROLSYSTEM_OUTPUT_ID, controlCan.c, 4);

		if (currentMaxonError) {
			uint16_to_2_bytes_t maxonErr;
			maxonErr.i = currentMaxonError;
			PRINT("MAXONERR: %d\n", currentMaxonError);
			canManager->writeMaxonSpecificCanError(maxonErr.c[0], maxonErr.c[1]);
			maxonMotor->resetFault(currentMaxonError);
		}

		// Wait for the pitch to be updated
		Thread::signal_wait(0x3);
	}

}
/*
 * main.cpp
 *
 *  Created on: Feb 28, 2014
 *      Author: boris & Bart
 */

#include <cmath>
#include <vector>
#include "mbed.h"
#include "rtos.h"
//#include "arm_math.h"
//#include "math_helper.h"

#include "MODSERIAL/MODSERIAL.h"

#include "CanManager.h"
#include "config.h"
#include "Watchdog.h"
#include "utilities/utilities.h"
#include "CANHardwareAcceptanceFilter.h"
#include "MaxonMotor.h"
#include "Flash.h"
#include "HeightSensor.h"
#include "Xsens/Xsens.h"
#include "Xsens/XsensDataParser.h"
#include "Xsens/XsensErrorParser.h"
#include "Xsens/XsensDataInterpreter.h"
#include "kalman/kalman.h"
#include "MaxonDriver.h"
#include "SenixManager.h"
#include "SteerManager.h"

//#include "Logger.h"

extern "C" void mbed_reset();
using namespace sbt;

 extern Serial pc;
extern Mutex pcMutex;

MODSERIAL xsensSerial(p17, p18, 100, XSENS_READ_BUFF_SIZE); // tx, rx
Xsens xsens(&xsensSerial);


CAN canBus(p30, p29);
CAN canMaxon(p9, p10);
Flash flash;

MODSERIAL heightSensor1Serial(p26, p25, 100, 100); // tx, rx
MODSERIAL heightSensor2Serial(p28, p27, 100, 100); // tx, rx
HeightSensor heightSensor1(&heightSensor1Serial,1);
HeightSensor heightSensor2(&heightSensor2Serial,2);

MaxonDriver *maxonDriver1;
MaxonDriver *maxonDriver2;

Thread *maxonDriver1Thread;
Thread *maxonDriver2Thread;

SteerManager *steerManager;

Thread *serialUserThread;

CanManager canManager(&canBus);
CanManager maxonCanManager(&canMaxon);

int tmp;

float senixHeight = 0;
float weirdVelocityMeasurement = 0.0f;


//States
float 	h, h_ref, q, theta, height1, height2, h_senix, V, a_z;

//statusBytes
uint16_t statusFront;

//Indicates whether hieght control is turned on.
bool heightOn = false;
float pitch_ref = 0;
uint16_t Status;
uint16_t currentMaxonError;
float previousVelocity = 0;

Watchdog watchdog;

//Data structures
//control_gains_t gains;
control_gains_t gains;

//Rtos variables
Thread *(xsens_TID);
Thread *(senix_TID);
Thread *(fly_TID);
Thread *(hull_TID);

osThreadId mainTID;
osThreadId xsensTID;
osThreadId senixTID;
osThreadId modeTID;

Mutex xsens_mutex;
Mutex xsens_flag_mutex;

Mutex senix_mutex;

SenixManager *senixManager;
Ticker *senixHeightOnPulseTicker;
Ticker *senixHeightOffPulseTicker;

//data.K_p * data.K_phi * (data.phi_ref - phi) - data.p);
typedef struct {
	float K_q = 0.019f; 		// pitch rate gain TODO make this gain configurable
	float K_h = .19f;		// height gain TODO make this gain configurable
	float K_theta = 1.5f;	// pitch gain TODO make this gain configurable
	float h_ref = 10.0f; 	// The height reference  (m)
	float h = 9.5f; 		// The actual height (m)
	float q = 0.1f; 		// The actual pitch rate (rad/s)
	float theta;			// actual pitch
	float alpha_eq = 0.0154f; // Angle of attack equilibrium point (rad)
	float K_p = 0.1f; 		// Roll rate gain
	float K_phi = 0.55f; 	// Roll angle gain
	float phi_ref = 0.0f; 	// Desired roll angle (rad)
	float phi = 0.0f;		// Measured roll angle (rad)
	float p = 0.0f;			// Measured roll rate (rad/s)
	float v = 0.0f; 		// Measured boat velocity (m/s)
	float r = 0.0f;			// Measured yaw rate (rad/s)
} PIDData;

PIDData pidData;

typedef struct {
	float ddiff;
	float d_req;
} PIDOut;

PIDOut pidOut;

//Reset the entire system
void systemReset(){
	maxonDriver1->reset();
	maxonDriver2->reset();
	mbed_reset();
}

/**
 * Setup hardware acceptance filter
 * filter CAN signals
 */
void setupHardwareAcceptanceFilter() {
	for(int i=0; i<TOTAL_ACCEPTED_CAN_IDS; i++) {
		CAN2_wrFilter(ACCEPTED_CAN_IDS[i]);
	}
}

void checkCanBuffer(){
	CANMessage m;
	while(!canManager.canBusBuffer->isEmpty()) {
		canManager.canBusBuffer->read(m);
		if (m.id == REPORT_TO_CAN_CONTROL_SYSTEM_HEIGHT_REAR_ID) {
			height2 = toFloat( m.data[0], m.data[1], m.data[2], m.data[3]);
			//PRINT("h2 = %f \t", height2);
		} else if (m.id == REPORT_TO_CAN_VELO_ID){
			V = (float) (toUint32( m.data[3], m.data[2], m.data[1], m.data[0])) *0.001/MS_TO_KMH;
			//PRINT("V = %f\n", V);
		} else if (m.id == STEER_INPUTS_ID){
			steerManager->parseCanMessage(&m);
		} else if(m.id == RESET_EPOS2_FRONT_ID){
			maxonDriver1->maxonMotor->resetEPOS2();
			maxonDriver2->maxonMotor->resetEPOS2();//TODO use separate commands for these?
		} else if(m.id == RESET_EPOS2_REAR_ID){
			maxonDriver1->maxonMotor->resetEPOS2();
			maxonDriver2->maxonMotor->resetEPOS2();
		} else if(m.id == RESET_SYSTEM_ID){
			systemReset();
		}
		else {
			flash.parseCanMessage(m);
		}
	}
}

/*
 * When a message comes in on the general CAN bus
 */
void canBusIsr() {
	CANMessage m;
	if(canBus.read(m)) {
		canManager.canBusBuffer->insert(m);
	}
	checkCanBuffer();
}

void pidInputsCANReport(CanManager *canManager, PIDData *data) {
	two_int8_to_2_bytes_t pidMessage;
	pidMessage.i[0] = (int8_t)(data->h_ref*255.0f);
	pidMessage.i[1] = (int8_t)(data->phi_ref * 255.0f);
	int canId = REPORT_TO_CAN_PID_INPUTS;
	canManager->writeCan(canId, pidMessage.c, 2);
}

void pidSensorsCANReport(CanManager *canManager, PIDData *data) {
	eight_int8_to_8_bytes_t pidMessage;
	pidMessage.i[0] = (int8_t)(data->phi*326.93f);
	pidMessage.i[1] = (int8_t)(data->h * 255.0f);
	pidMessage.i[2] = (int16_t)(data->q * 100000.0f);
	pidMessage.i[4] = (int16_t)(data->p * 100000.0f);
	pidMessage.i[6] = (int16_t)(data->r * 100000.0f);
	int canId = REPORT_TO_CAN_PID_SENSORS;
	canManager->writeCan(canId, pidMessage.c, 8);
}

void pidGainsCANReport(CanManager *canManager, PIDData *data) {
	five_int8_to_5_bytes_t pidMessage;
	pidMessage.i[0] = (int8_t)(data->K_p * 100.0f);
	pidMessage.i[1] = (int8_t)(data->K_h * 10.0f);
	pidMessage.i[2] = (int8_t)(data->alpha_eq * 100.0f);
	pidMessage.i[3] = (int8_t)(data->K_q * 100.0f);
	pidMessage.i[4] = (int8_t)(data->K_phi * 10.0f);
	int canId = REPORT_TO_CAN_PID_GAINS;
	canManager->writeCan(canId, pidMessage.c, 5);
}

void pidOutReport(CanManager *canManager, PIDOut *data) {
	two_int16_to_4_bytes_t pidMessage;
	pidMessage.i[0] = (int16_t)(data->d_req * 1000.0f);
	pidMessage.i[1] = (int16_t)(data->ddiff * 1000.0f);

	int canId = REPORT_TO_CAN_PID_OUT;
	canManager->writeCan(canId, pidMessage.c, 5);
}

void reportEverything() {
	XsensDataInterpreter interpreter = XsensDataInterpreter(&(xsens.xsensData));


	two_int32_to_8_bytes_t lat_lon;
	lat_lon.i[0] = (int32_t)(xsens.xsensData.latlon.lat*100000);
	lat_lon.i[1] = (int32_t)(xsens.xsensData.latlon.lon*100000);
	canManager.writeCan(REPORT_TO_CAN_LAT_LON_ID, lat_lon.c, 8);

	maxonDriver1->CANReport(&canManager);
	maxonDriver2->CANReport(&canManager);

	two_int32_to_8_bytes_t alt_velX;
	alt_velX.i[0] = (int32_t)(xsens.xsensData.altitude*1000);
	alt_velX.i[1] = (int32_t)(xsens.xsensData.velocity);
	canManager.writeCan(REPORT_TO_CAN_ALT_VELX_ID, alt_velX.c, 8);

	maxonDriver1->CANReport(&canManager);
	maxonDriver2->CANReport(&canManager);

	heightSensor1.CANReport(&canManager, &heightSensor2);

	heightSensor1.CANReportStatistics(&canManager);
	heightSensor2.CANReportStatistics(&canManager);

	pidInputsCANReport(&canManager, &pidData);
	pidSensorsCANReport(&canManager, &pidData);
	pidGainsCANReport(&canManager, &pidData);
	pidOutReport(&canManager, &pidOut);

	steerManager->CANReport(&canManager);

	two_int32_to_8_bytes_t accX_accY;
	accX_accY.i[0] = (int32_t)(xsens.xsensData.acceleration.x*1000);
	accX_accY.i[1] = (int32_t)(xsens.xsensData.acceleration.y*1000);
	canManager.writeCan(REPORT_TO_CAN_ACCX_ACCY_ID, accX_accY.c, 8);

	two_int32_to_8_bytes_t accZ_pitch;
	accZ_pitch.i[0] = (int32_t)(xsens.xsensData.acceleration.z*1000);
	accZ_pitch.i[1] = (int32_t)(interpreter.getPitch()*1000);
	canManager.writeCan(REPORT_TO_CAN_ACCZ_PITCH_ID, accZ_pitch.c, 8);

	two_int32_to_8_bytes_t roll_yaw;
	roll_yaw.i[0] = (int32_t)(interpreter.getRoll()*1000);
	roll_yaw.i[1] = (int32_t)(xsens.xsensData.rotation.yaw*1000);
	canManager.writeCan(REPORT_TO_CAN_ROLL_YAW_ID, roll_yaw.c, 8);

	two_int32_to_8_bytes_t rotX_rotY;
	rotX_rotY.i[0] = (int32_t)(xsens.xsensData.rot.x*1000);
	rotX_rotY.i[1] = (int32_t)(xsens.xsensData.rot.y*1000);
	canManager.writeCan(REPORT_TO_CAN_ROTX_ROTY_ID, rotX_rotY.c, 8);

	int32_to_4_bytes_t rotZ;
	rotZ.i = (int32_t)(xsens.xsensData.rot.z*1000);
	canManager.writeCan(REPORT_TO_CAN_ROTZ_ID, rotZ.c, 4);

	time_to_8_bytes_t time;
	time.year 	= xsens.xsensData.time.year;
	time.month 	= xsens.xsensData.time.month;
	time.day 	= xsens.xsensData.time.day;
	time.hour 	= xsens.xsensData.time.hour;
	time.minute = xsens.xsensData.time.minute;
	time.second	= xsens.xsensData.time.second;
	canManager.writeCan(REPORT_TO_CAN_TIME_ID, time.c, 8);

	two_int32_to_8_bytes_t velo;
	float v = xsens.xsensData.velocity * MS_TO_KMH * 1000;
	float v_xy = xsens.xsensData.velocity_xy * MS_TO_KMH * 1000;
	velo.i[0] = (int32_t)v;
	velo.i[1] = (int32_t)v_xy;
	canManager.writeCan(REPORT_TO_CAN_VELO_ID, velo.c, 8);

	eight_int8_to_8_bytes_t sensorPanic;
	sensorPanic.i[0] = (int32_t)weirdVelocityMeasurement;
	sensorPanic.i[4] = (int16_t)maxonDriver1->reachedBoundCount;
	sensorPanic.i[6] = (int16_t)maxonDriver2->reachedBoundCount;
	canManager.writeCan(REPORT_TO_CAN_SENSORPANIC, sensorPanic.c, 8);
	if (weirdVelocityMeasurement > 0) {
		weirdVelocityMeasurement = 0;
	}

}

void reportToCanHeightOn() {
	if (steerManager->steerData->heightOn) {
		reportEverything();
	}
}

/*
 * Report data to can
 */
void reportToCan() {
	if (!steerManager->steerData->heightOn) {
		reportEverything();
	}
}


void getGains(control_gains_t *controlGains){
    controlGains->K_h        = flash.getK_hGain();
    controlGains->K_theta    = flash.getK_thetaGain();
    controlGains->K_q        = flash.getK_qGain();
	controlGains->baseGain	 = flash.getBaseGain();
}

void getGain(float *K_v){
	*K_v = flash.getK_vGain();
}

void errorState(uint8_t exception){
	int cnt = 10;
	while(cnt > 0){
#ifdef MBED_FRONT
		canManager.writeCanError(CanErrorCategory::maxonmotor1, exception);
#else
		canManager.writeCanError(CanErrorCategory::maxonmotor2, exception);
#endif

		wait_ms(100);
		cnt--;
	}
	mbed_reset();

}

void kalman(float a_z ,float h_senix){
	float htemp = kalman_filter(a_z ,h_senix);
	//This is a safetycheck to check whether the kalman filter overflows.
	//If it does, re-initiliaze it.
	uint16_t counter = 0;
	if (-1<htemp and htemp<2.5){
		clearBit(&Status, StatusKalman);
		h = htemp;	
		counter =0;
	}else{
		//Reset Kalman
		
		setBit(&Status, StatusKalman);
		//kal_init();
		counter++;
		if(counter>1000){
			kal_init();
			counter = 0;
		}
	}
}

void checkGPS(){
	if(V==0){
		clearBit(&Status, StatusGPS);
	}
	else if(V!=0){
		setBit(&Status, StatusGPS);
	}
}

//---------------------Sensors-----------------------------------------------------------------------------------'
//Fuctions for Senix readings
void senixRxISR1(MODSERIAL_IRQ_INFO *info){
	/*
	 * This function releases the semaphore flag when the UART receives the first data message from the sensor
	 */
	heightSensor1.sem->release();

}
void senixRxISR2(MODSERIAL_IRQ_INFO *info){
	/*
	 * This function releases the semaphore flag when the UART receives the first data message from the sensor
	 */
	heightSensor2.sem->release();
}

void xsensRxISR(MODSERIAL_IRQ_INFO *info){
	/*
	 * This function releases the semaphore when the UART receives the first data message from the sensor
	 */
	
	xsens.sem->release();
}

void canMaxonIsr() {

	CANMessage m;
	if(canMaxon.read(m)) {
		if (1 & m.id) {//maxonDriver1->nodeID
			//maxon 1
			maxonDriver1->canIsr(m);
		} else if (2 & m.id) { //maxonDriver2->nodeID
			maxonDriver2->canIsr(m);
		} else {
			PRINT("can maxon isr: invalid sender address %04x.\n",m.id);
			//TODO write an error message to can that the address of sender is invalid
		}
	}
}



void Xsens_thread(void const *TID){
	xsens.thread(TID);
}


float getRollCorrect(){
	/*
	This function increases the reference height of the boat depending on the roll angle
	to 'fool' the control system in corners
	*/
	XsensDataInterpreter interpreter = XsensDataInterpreter(&(xsens.xsensData));
	float roll = abs(interpreter.getRoll());
	float ROLL_LOWER_THRESHOLD = 3.5, ROLL_HIGHER_THRESHOLD = 10;
	float factor = 0.008375;
	if(roll>ROLL_LOWER_THRESHOLD and roll<ROLL_HIGHER_THRESHOLD){
		float correct = (roll-ROLL_LOWER_THRESHOLD) * factor;// * pitch_ref;
		setBit(&Status, StatusRoll);
		return correct;
	} else if(roll>ROLL_HIGHER_THRESHOLD){
		float correct =(ROLL_HIGHER_THRESHOLD - ROLL_LOWER_THRESHOLD) * factor;// * pitch_ref;
		setBit(&Status, StatusRoll);
		return correct;
	}else{
		clearBit(&Status, StatusRoll);
		return 0;
	}
}



void printData(){
	PRINT("H1 = %f.3,\t H2 = %f.3\n", height1, height2);
	PRINT("V = %f,\t theta = %f,\t q = %f,\t a_z = %f", V, theta, q, a_z);
}


void maxon1Thread(void const *TID) {
	maxonDriver1->thread(TID);
}

void maxon2Thread(void const *TID) {
	maxonDriver2->thread(TID);
}

void senixThread1(void const *TID) {
	heightSensor1.thread(TID);
}

void senixThread2(void const *TID) {
	heightSensor2.thread(TID);
}


void evaluatePID(PIDData data) {

	if (!maxonDriver1->isReady() || !maxonDriver2->isReady()) {
		// Wait until both maxonsmotors are ready.
		PRINT("still waiting %d %d\n",maxonDriver1->isReady(), maxonDriver2->isReady());
		wait(1);
		return;
	}

	float d_req = 0;
	if (steerManager->steerData->heightOn) {
		//d_req = K_q * (K_h * (h_ref - h) - q)
		d_req = data.K_h * (data.h_ref - data.h) - data.K_q * data.q + data.alpha_eq;
//		d_req = data.K_h * (data.h_ref - data.h) + data.alpha_eq;
//		d_req = data.K_h * (data.h_ref - data.h) + data.alpha_eq;
//		d_req =  data.K_q * ( data.K_theta * ( data.K_h * (data.h_ref - data.h) - data.theta) - data.q);
//		d_req = data.alpha_eq + data.K_h * (data.h_ref - data.h);
	} else {
		d_req = MAXON_ZEROLIFT_RAD;
	}

//	float ddiff = data.K_phi * (data.phi_ref - data.phi);
	float ddiff = data.K_phi * (data.phi_ref - data.phi) - data.K_p * data.p;
	float boatPitch = data.theta;

	float inverter = 1; // 1 or -1;
	float d_req1 = d_req - boatPitch - ddiff * inverter;
	float d_req2 = d_req - boatPitch + ddiff * inverter;

	maxonDriver1->setPitchRad(d_req1);
	maxonDriver2->setPitchRad(d_req2);

	pidOut.d_req = d_req;
	pidOut.ddiff = ddiff;
}


void processHeight(void const *TID) {
	senixHeight = 0;
	float prevSenixHeight = 0;
	int senixFails = 0;
	while (1) {

		// Wait for new information
//		Thread::signal_wait(0x1);

		wait_us(100);


		if (heightSensor1.getAge() < 500 && heightSensor2.getAge() < 500) {
			float a = heightSensor1.getHeight();
			float b = heightSensor2.getHeight();
			senixHeight = senixHeight * .8f + (a / 2 + b / 2) * .2f;
//			PRINT("a: %f, b: %f\n", a, b);
		} else if (heightSensor1.getAge() < 500) {
			senixHeight = senixHeight * .8f + heightSensor1.getHeight() * .2f;
//			PRINT("a: %f\n", senixHeight);
			//TODO report communication lost heightsensor2
		} else if (heightSensor2.getAge() < 500) {
//			PRINT("b: %f\n", senixHeight);
			senixHeight = senixHeight * .8f + heightSensor2.getHeight() * .2f;
			//TODO report communication lost heightsensor1
		}



		xsens.xsensData_mutex->lock();
//			senixHeight = senixHeight * cos((xsens.xsensData.rotation.roll + XSENS_ROLL_BIAS_CORRECTION) /180*PI);

			prevSenixHeight = senixHeight;
			pidData.h = senixHeight;

			if (xsens.xsensData.velocity_xy > 17.0f) {
				// We don't trust the velocity sensor if the velocity is greater than 60km/h
				// The old velocity value is being  used
				weirdVelocityMeasurement = max(weirdVelocityMeasurement, xsens.xsensData.velocity_xy);
			} else {
				pidData.v = xsens.xsensData.velocity_xy;
			}
			pidData.q = xsens.xsensData.rot.y;
			pidData.phi = (xsens.xsensData.rotation.roll + XSENS_ROLL_BIAS_CORRECTION)/180*PI;
			pidData.p = xsens.xsensData.rot.x;
			pidData.theta = xsens.xsensData.rotation.pitch/180*PI;
			pidData.r = pidData.r * .6f + xsens.xsensData.rot.z * .4f;
		xsens.xsensData_mutex->unlock();

//		pidData.phi_ref = max(-MAXIMUM_PHI_REF, min(MAXIMUM_PHI_REF, atan((pidData.v * pidData.r) / 9.81f) * 0.75f ));
		pidData.phi_ref = max(-MAXIMUM_PHI_REF, min(MAXIMUM_PHI_REF, (pidData.v * pidData.r) / 9.81f * 0.75f ));

//		PRINT("phi_ref with velocity:300ms -> %f\n",atan((300 * xsens.xsensData.rot.z) / 9.81f));
//		PRINT("rpy rr pr yr%f\t  %f\t  %f\t  %f\t %f\t %f\n", xsens.xsensData.rotation.roll/180*PI, xsens.xsensData.rotation.pitch/180*PI, xsens.xsensData.rotation.yaw/180*PI,
//			xsens.xsensData.rot.x, xsens.xsensData.rot.y, xsens.xsensData.rot.z);

		pidData.h_ref = steerManager->getReferenceHeight();

		steerManager->tryCalibrateSteerAngle(&xsens);


		// Update the alpha_eq based on current velocity
		float v = max(pidData.v, 6);
		if (max(v, previousVelocity) - min(v, previousVelocity) > 0.03f) {
//			pidData.alpha_eq = 0.00007f * pow(v, 4) - 0.0033f * pow(v, 3) + 0.0575f * pow(v, 2) - 0.4476f * v + 1.3332f;
			pidData.alpha_eq = 0.000075f * pow(v, 4) - 0.0035f * pow(v, 3) + 0.06f * pow(v, 2) - 0.47f * v + 1.4f;
			previousVelocity = v;
		}

		evaluatePID(pidData);

		// This allows rolling when heightcontrol is off
		maxonDriver1->setSteerRoll(steerManager->steerData->roll);
		maxonDriver2->setSteerRoll(steerManager->steerData->roll);

		watchdog.kick();
	}
}

void senix_tick_heightOff () {
	senixManager->senix_tick_heightOff();
}

void senix_tick_heightOn () {
	senixManager->senix_tick_heightOn();
}

void senixManagerThread(void const *TID) {
	senixManager->thread();
}

//Main--------------------------------------------------------------------------------------------------------------
int main() {
#ifdef DEBUG
	PRINT("\r\nSolar Boat Height Control 2017.\n");
#endif
	mainTID = osThreadGetId();

	//Setup watchdog for setup procedure
	watchdog.kick(180.0);

	flash.loadFromFiles();
	//flash.printValues();

	// set can frequency
	canBus.frequency(CAN_FREQUENCY);
	canMaxon.frequency(CAN_FREQUENCY);
	
	// add can interrupt handler
	canBus.attach(&canBusIsr);
	canMaxon.attach(&canMaxonIsr);

	heightSensor1Serial.attach(&senixRxISR1, MODSERIAL::RxIrq);
	heightSensor2Serial.attach(&senixRxISR2, MODSERIAL::RxIrq);

	
	Thread processThread(&processHeight, NULL, osPriorityNormal, DEFAULT_STACK_SIZE);
	heightSensor1.processThread = &processThread;
	heightSensor2.processThread = &processThread;

	Thread senix1Thread(&senixThread1, NULL ,osPriorityNormal, SENIX_STACK_SIZE);
	Thread senix2Thread(&senixThread2, NULL ,osPriorityNormal, SENIX_STACK_SIZE);

	Thread xsens_t(&Xsens_thread, NULL ,osPriorityNormal, XSENS_STACK_SIZE);
	xsens_TID = &xsens_t;
	xsens.processThread = &processThread;
	xsensSerial.attach(&xsensRxISR, MODSERIAL::RxIrq);

	maxonDriver1 = new MaxonDriver(&maxonCanManager, &canManager, 1);
	maxonDriver2 = new MaxonDriver(&maxonCanManager, &canManager, 2);

	maxonDriver1Thread = new Thread(&maxon1Thread, NULL ,osPriorityNormal, SENIX_STACK_SIZE);
	maxonDriver2Thread = new Thread(&maxon2Thread, NULL ,osPriorityNormal, SENIX_STACK_SIZE);

	maxonDriver1->setOwnThread(maxonDriver1Thread);
	maxonDriver2->setOwnThread(maxonDriver2Thread);

	steerManager = new SteerManager();

	setupHardwareAcceptanceFilter();

	//Initialize Kalman filter
	//kal_init();

	//Tickers
	Ticker reportCan, reportCanHeightOn; //printdata;
	reportCan.attach(reportToCan, 1);
	reportCanHeightOn.attach(reportToCanHeightOn, 0.05);

//	Thread *senixManager_Thread = new Thread(&senixManagerThread, NULL, osPriorityHigh, SMALL_STACK_SIZE);

	senixManager = new SenixManager();
	senixManager->senixThread1 = &senix1Thread;
	senixManager->senixThread2 = &senix2Thread;
	senixManager->heightSensor1 = &heightSensor1;
	senixManager->heightSensor2 = &heightSensor2;
	senixManager->steerManager = steerManager;
//	senixManager->ownThread = senixManager_Thread;
	senixManager->processThread = &processThread;

	senixHeightOnPulseTicker = new Ticker();
	senixHeightOffPulseTicker = new Ticker();

	senixHeightOffPulseTicker->attach(senix_tick_heightOff,5);
	senixHeightOnPulseTicker->attach(senix_tick_heightOn, 0.05);

	watchdog.kick();

	while(1) {
		senixManager->thread();
	}


	// Wait forever
	osSignalWait(0x1, osWaitForever);
}

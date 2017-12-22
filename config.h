/*
 * main.h
 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "Xsens/Xsens.h"
#include "Flash.h"

#define MBED_FRONT								1
#define FEED_FORWARD_TIME						2.5 //seconds

#define CAN_FREQUENCY 							500000
#define XSENS_BAUD_RATE							460800
#define HEIGHT_SENSOR_FRONT_BAUD_RATE			57600
#define HEIGHT_SENSOR_BACK_BAUD_RATE			57600

#define CAN_HUNDRED_PERCENT_FACTOR				0.3921
#define CAN_ANGLE_FACTOR						1.4056

#define DISTANCE_BETWEEN_HEIGHT_SENSORS			5.62
#define HEIGHTSENSOR1_H							0.375
#define HEIGHTSENSOR2_H							0.195
#define TRESHOLD_ZERO_ABOVE_WATER				0.05

//Isr timers
#define REPORT_TO_CAN_WAIT						0.5
#define REPORT_STATUS_WAIT						0.5
#define STEER_ANGLE_RATIO						0.2582f

#define REPORT_TO_CAN_LAT_LON_ID					0x220
#define REPORT_TO_CAN_ALT_VELX_ID					0x221
#define REPORT_TO_CAN_VELY_VELZ_ID					0x222
#define REPORT_TO_CAN_ACCX_ACCY_ID					0x223
#define REPORT_TO_CAN_ACCZ_PITCH_ID					0x224
#define REPORT_TO_CAN_ROLL_YAW_ID					0x225
#define REPORT_TO_CAN_ROTX_ROTY_ID					0x226
#define REPORT_TO_CAN_ROTZ_ID						0x227
#define REPORT_TO_CAN_TIME_ID						0x228
#define REPORT_TO_CAN_HEIGHT_ID						0x229
#define REPORT_TO_CAN_VELOCITY_BOAT_ID				0x22A
#define REPORT_TO_CAN_VELO_ID						0X290
#define REPORT_TO_CAN_CONTROLSYSTEM_OUTPUT_ID		0x22B
#define WING_CONTROL_ERROR_CAN_ID					0x22C
#define REPORT_TO_CAN_DESIRED_HEIGHT_AUTO_VALUE_ID	0x22D
#define REPORT_TO_CAN_CURRENT_ANGLE_FRONT_ID		0x22E
#define REPORT_TO_CAN_CURRENT_ANGLE_REAR_ID			0x232
#define REPORT_TO_CAN_K_H_OUTPUT_ID					0x22F
#define REPORT_TO_CAN_K_THETA_OUTPUT_ID				0x230
#define REPORT_TO_CAN_K_Q_OUTPUT_ID					0x231
#define REPORT_TO_CAN_BASEGAIN_OUTPUT_ID			0x233

#define REPORT_TO_CAN_STATUS_FRONT_ID				0x279
#define REPORT_TO_CAN_CONTROL_SYSTEM_HEIGHT_ID 0x247
#define REPORT_TO_CAN_CONTROL_SYSTEM_HEIGHT_REAR_ID 0x248
#define REPORT_TO_CAN_STATUS_REAR_ID				0X27A

#define REPORT_TO_CAN_SENIX							0x500
#define REPORT_TO_CAN_SENIX_STATISTICS				0x501 // 0x501+0 for node 1, and 0x501+1 for node 2
#define REPORT_TO_CAN_MAXON							0x503 // 0x503+0 for node 1, and 0x503+1 for node 2
#define REPORT_TO_CAN_PID_SENSORS					0x505
#define REPORT_TO_CAN_PID_INPUTS					0x506
#define REPORT_TO_CAN_PID_GAINS						0x507
#define REPORT_TO_CAN_PID_OUT						0x508
#define UPDATE_FROM_CAN								0x509
#define REPORT_TO_CAN_STEERANGLE					0x50A
#define REPORT_TO_CAN_SENSORPANIC					0x50B

#define SENIX_REPORT_STATISTICS_INTERVAL			1000 * 60 * 2 // every two minutes


#define CAN_BUS_DESIRED_HEIGHT_ID 				0x240
#define REQUEST_DESIRED_HEIGHT_ID				0x242

#define STEER_INPUTS_ID					101

#define CAN_WRITE_DEFAULT_TIMEOUT_US			1500

#define DESIRED_HEIGHT_AUTO						0xFF

#define MS_TO_KMH									3.6
#define DESIRED_HEIGHT_AUTO_VELOCITY_MIN		6.318840579710144927536231884058
#define DESIRED_HEIGHT_AUTO_VELOCITY_MAX		10.956521739130434782608695652174
#define DESIRED_HEIGHT_AUTO_CONSTANT			0.129375
#define DESIRED_HEIGHT_MAX						0.6
#define MAXIMUM_PHI_REF							0.349f	// the maximum leaning angle is 20 degrees
#define STEER_TO_RUDDER_RATIO					0.1667f;

#define RESET_EPOS2_FRONT_ID						0x275
#define RESET_EPOS2_REAR_ID						0x276
#define RESET_SYSTEM_ID							0x277


#define PID_P_VELOCITY_FACTOR 		0.007 //With this value, the P factor is halved at top speed
#define PID_D_VELOCITY_FACTOR			0.075

#define SENIX_STACK_SIZE			1024
#define SMALL_STACK_SIZE			512

// MAXON calibration
#define MAXON_NEUTRAL_POSITION	 -360000
#define MAXON_MAX_NEGATIVE_ANGLE -620000
#define MAXON_MAX_POSITIVE_ANGLE 0
#define MAXON_NEXT_PITCH_MINIMAL_DISTANCE 500
#define MAXON_MAXRAD			 0.3371f
#define MAXON_MINRAD			 -0.2288f
#define MAXON_ZEROLIFT_RAD		 -0.04f
#define MAXON_TOTALRAD			 (MAXON_MAXRAD-MAXON_MINRAD)
#define MAXON_POSITION_MARGIN	 50000


//Statusbit layout
#define StatusGPS 		0
#define StatusHoming 	1
#define StatusMaxon 	2
#define StatusKalman 	3
#define StatusAutoMode	4
#define StatusRoll		5
#define StatusGPSLoss 	6


#ifdef MBED_FRONT
	#define TOTAL_ACCEPTED_CAN_IDS					16
	const uint32_t ACCEPTED_CAN_IDS[TOTAL_ACCEPTED_CAN_IDS] = {
		K_H_GAIN_ID,
		K_THETA_GAIN_ID,
		K_Q_GAIN_ID,

		KAL_X_ID,
		KAL_H_ID,

		FRONT_ACTUATOR_ACCELERATION_ID,
		FRONT_ACTUATOR_VELOCITY_ID,
		FRONT_ACTUATOR_MAX_FOLLOWING_ERROR_ID,

		FRONT_MAX_ANGLE_ID,
		FRONT_NEUTRAL_ANGLE_ID,

		REPORT_TO_CAN_CONTROL_SYSTEM_HEIGHT_REAR_ID,
		CAN_BUS_DESIRED_HEIGHT_ID,

		STEER_INPUTS_ID,

		RESET_EPOS2_FRONT_ID,
		RESET_SYSTEM_ID,
		BASE_GAIN_ID
	};
#else
	#define TOTAL_ACCEPTED_CAN_IDS				11
	const uint32_t ACCEPTED_CAN_IDS[TOTAL_ACCEPTED_CAN_IDS] = {
			REPORT_TO_CAN_VELOCITY_BOAT_ID,
			K_V_GAIN_ID,

			REAR_ACTUATOR_ACCELERATION,
			REAR_VELOCITY_ID,
			REAR_ACTUATOR_MAX_FOLLOWING_ERROR_ID,

			REAR_MAX_ANGLE_ID,
			REAR_NEUTRAL_ANGLE_ID,

			STEER_INPUTS_ID,
			DASHBOARD_PITCH_ID,

			RESET_EPOS2_REAR_ID,
			RESET_SYSTEM_ID
	};
#endif

typedef struct {
	float P;
	float I;
	float D;
	float DD;
} process_values_t;

typedef struct {
	float K_h;  //0.4
	float K_theta; 	//0.5
	float K_q;  //0.8
	uint32_t baseGain;	//25000
} control_gains_t;

typedef struct {
	float steerAngle = 0.0f;
	bool backward = 0;
	float height = 0.0f;
	bool heightOn = false;
	float pitch = 0;
	float roll = 0;
	bool screen =0;
	bool talk = 0;
	float temperature = 0;
	float throttle = 0.0f;
} SteerData;

/* ISR */
void canBusIsr();
void canMaxonIsr();
void reportToCanIsr();
void controlMotorFlagIsr();
void senixReadIsr();

/* GENERAL CAN */
void setupHardwareAcceptanceFilter();
void reportToCan(process_values_t* processValues);
void requestDesiredHeight();

/* SENSORS */
void heightSensorSetup();
void heightSensorUpdateParameters();
XsensException readXsens();
HeightSensorException readHeightSensor1();
HeightSensorException readHeightSensor2();
void updateHeightSensorParamaters();

/* CONTROL SYSTEM */
void setDesiredHeightAuto();
void getGains(control_gains_t *controlGains);
float controller(XsensException, HeightSensorException, HeightSensorException);
float getPID(process_values_t* proccessValues);

/* MOTOR */
void controlMotorFront(control_gains_t* controlgains, float feedback);
void maxonMotorUpdateParameters();
void maxonSetup();


#endif /* MAIN_H_ */

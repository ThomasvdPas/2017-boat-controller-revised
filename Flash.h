/*
 * Flash.h
 *
 *  Created on: 22 jan. 2014
 *      Author: boris
 */


#ifndef FLASH_H_
#define FLASH_H_

#include <string>
#include "mbed.h"

// gains
#define K_H_GAIN_FILENAME										"/local/K_h_gain.txt"
#define K_THETA_GAIN_FILENAME									"/local/K_theta_gain.txt"
#define K_Q_GAIN_FILENAME										"/local/K_q_gain.txt"
#define K_V_GAIN_FILENAME										"/local/K_v_gain.txt"
#define BASE_GAIN_FILENAME										"/local/baseGain.txt"

// Kalman filter
#define KAL_X_FILENAME											"/local/Kal_x.txt"
#define KAL_H_FILENAME											"/local/Kal_h.txt"

// height sensor
#define HEIGHT_SENSOR_NUMBER_OF_SAMPLES_TO_BE_AVERAGED_FILENAME	"/local/samples.txt"
#define HEIGHT_SENSOR_INTERVAL_FILENAME							"/local/interval.txt"
#define HEIGHT_SENSOR_TRANSMITTED_PULSES_FILENAME				"/local/pulses.txt"

// actuator
#define ACTUATOR_ACCELERATION_FILENAME							"/local/acc.txt"
#define ACTUATOR_VELOCITY_FILENAME								"/local/velocity.txt"
#define ACTUATOR_MAX_FOLLOWING_ERROR_FILENAME					"/local/follow.txt"

// max angle
#define MAX_ANGLE_FILENAME										"/local/max_angl.txt"
#define NEUTRAL_ANGLE_FILENAME									"/local/neu_angl.txt"

#define K_H_GAIN_ID 											0x260
#define K_THETA_GAIN_ID 										0x261
#define K_Q_GAIN_ID 											0x262
#define K_V_GAIN_ID												0x263

#define KAL_X_ID												0x280
#define KAL_H_ID												0x281

#define HEIGHT_SENSOR_NUMBER_OF_SAMPLES_TO_BE_AVERAGED_ID		0x266
#define HEIGHT_SENSOR_INTERVAL_ID 								0x267
#define HEIGHT_SENSOR_TRANSMITTED_PULSES_ID						0x268

#define FRONT_ACTUATOR_ACCELERATION_ID		 						0x269
#define FRONT_ACTUATOR_VELOCITY_ID		 							0x26A
#define FRONT_ACTUATOR_MAX_FOLLOWING_ERROR_ID			 			0x26B
#define REAR_ACTUATOR_ACCELERATION_ID								0x270
#define REAR_ACTUATOR_VELOCITY_ID									0x271
#define REAR_ACTUATOR_MAX_FOLLOWING_ERROR_ID						0x272

#define FRONT_NEUTRAL_ANGLE_ID			 						0x26C
#define FRONT_MAX_ANGLE_ID			 							0x26D
#define REAR_NEUTRAL_ANGLE_ID									0x273
#define REAR_MAX_ANGLE_ID										0x274
#define BASE_GAIN_ID											0x278


class Flash {
private:
	float K_hGain, K_thetaGain, K_qGain, K_vGain;
	uint32_t baseGain;
	float kal_x, kal_h;
	//float maxAngle; //, neutralAngle;
	
	uint32_t neutralAngle, maxAngle;

	uint8_t		heightSensorNumberOfSamplesToBeAveraged;
	uint32_t	heightSensorInterval;
	uint16_t	heightSensorTransmittedPulses;

	uint32_t	actuatorAcceleration,
				actuatorVelocity,
				actuatorMaxFollowingError;

	float readCanData(uint8_t* data);
	template<typename writeToFileType>
	void writeToFile(std::string filename, writeToFileType value);
	template<typename readFromFileType>
	readFromFileType readFromFile(std::string filename);
public:
	Flash();
	void loadFromFiles();
	void parseCanMessage(CANMessage msg);

	float getK_hGain();
	float getK_thetaGain();
	float getK_qGain();
	float getK_vGain();
	uint32_t getBaseGain();
	float getKal_x();
	float getKal_h();

	uint32_t getMaxAngle();
	uint32_t getNeutralAngle();//float getNeutralAngle();


	void setK_hGain(float gain);
	void setK_thetaGain(float gain);
	void setK_qGain(float gain);
	void setK_vGain(float gain);
	void setBaseGain(uint32_t gain);
	void setKal_x(float kal);
	void setKal_h(float kal);
	void setMaxAngle(uint32_t maxAngle); //void setMaxAngle(float maxAngle);
	void setNeutralAngle(uint32_t neutralAngle); //void setNeutralAngle(float neutralAngle);
	void setHeightSensorNumberOfSamplesToBeAveraged(uint8_t samples);
	void setHeightSensorInterval(uint32_t samples);
	void setHeightSensorTransmittedPulses(uint16_t samples);
	void setActuatorAcceleration(uint32_t samples);
	void setActuatorVelocity(uint32_t samples);
	void setActuatorMaxFollowingError(uint32_t samples);
	uint32_t getActuatorAcceleration() const;
	uint32_t getActuatorMaxFollowingError() const;
	uint32_t getActuatorVelocity() const;
	uint32_t getHeightSensorInterval() const;
	uint8_t getHeightSensorNumberOfSamplesToBeAveraged() const;
	uint16_t getHeightSensorTransmittedPulses() const;

	void printValues();
};

#endif /* FLASH_H_ */

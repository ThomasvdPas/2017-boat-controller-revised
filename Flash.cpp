/*
 * Flash.cpp
 *
 *  Created on: 22 jan. 2014
 *      Author: boris
 */

#include <string.h>
#include <iostream>
#include <stdlib.h>
#include "mbed.h"
#include "utilities/utilities.h"
#include "Flash.h"
#include "rtos.h"

extern Serial pc;
extern Mutex pcMutex;

LocalFileSystem local("local");

Flash::Flash():
		K_hGain(0),
		K_thetaGain(0),
		K_qGain(0),
		K_vGain(0),
		baseGain(0),
		kal_x(0),		//Trust factor xsens
		kal_h(0),		//Trust factor senix
		neutralAngle(0),
		maxAngle(0),
		heightSensorNumberOfSamplesToBeAveraged(0),
		heightSensorInterval(0),
		heightSensorTransmittedPulses(0),
		actuatorAcceleration(0),
		actuatorVelocity(0),
		actuatorMaxFollowingError(0)
{
}

void Flash::loadFromFiles() {
	/** load gains from the file system **/
	K_hGain = readFromFile<float>(K_H_GAIN_FILENAME);
	K_thetaGain = readFromFile<float>(K_THETA_GAIN_FILENAME);
	K_qGain = readFromFile<float>(K_Q_GAIN_FILENAME);
	K_vGain = readFromFile<float>(K_V_GAIN_FILENAME);
	baseGain = readFromFile<uint32_t>(BASE_GAIN_FILENAME);
	PRINT("BASE %i", baseGain);

	kal_x = readFromFile<float>(KAL_X_FILENAME);
	kal_h = readFromFile<float>(KAL_H_FILENAME);

	maxAngle = readFromFile<uint32_t>(MAX_ANGLE_FILENAME);
	neutralAngle = readFromFile<uint32_t>(NEUTRAL_ANGLE_FILENAME);//readFromFile<float>(NEUTRAL_ANGLE_FILENAME);

	heightSensorNumberOfSamplesToBeAveraged = readFromFile<uint8_t>(HEIGHT_SENSOR_NUMBER_OF_SAMPLES_TO_BE_AVERAGED_FILENAME);
	heightSensorInterval = readFromFile<uint32_t>(HEIGHT_SENSOR_INTERVAL_FILENAME);
	heightSensorTransmittedPulses = readFromFile<uint16_t>(HEIGHT_SENSOR_TRANSMITTED_PULSES_FILENAME);

	actuatorAcceleration = readFromFile<uint32_t>(ACTUATOR_ACCELERATION_FILENAME);
	actuatorVelocity = readFromFile<uint32_t>(ACTUATOR_VELOCITY_FILENAME);
	actuatorMaxFollowingError = readFromFile<uint32_t>(ACTUATOR_MAX_FOLLOWING_ERROR_FILENAME);
}


float Flash::getK_hGain() {
	return K_hGain;
}
float Flash::getK_thetaGain() {
	return K_thetaGain;
}
float Flash::getK_qGain() {
	return K_qGain;
}
float Flash::getK_vGain(){
	return K_vGain;
}
uint32_t Flash::getBaseGain(){
	return baseGain;
}
float Flash::getKal_x(){
	return kal_x;
}
float Flash::getKal_h(){
	return kal_h;
}
uint32_t Flash::getMaxAngle() {
	return maxAngle;
}
uint32_t Flash::getNeutralAngle(){//float Flash::getNeutralAngle() {
	return neutralAngle;
}
uint32_t Flash::getActuatorAcceleration() const {
	return actuatorAcceleration;
}

uint32_t Flash::getActuatorMaxFollowingError() const {
	return actuatorMaxFollowingError;
}

uint32_t Flash::getActuatorVelocity() const {
	return actuatorVelocity;
}

uint32_t Flash::getHeightSensorInterval() const {
	return heightSensorInterval;
}

uint8_t Flash::getHeightSensorNumberOfSamplesToBeAveraged() const {
	return heightSensorNumberOfSamplesToBeAveraged;
}

uint16_t Flash::getHeightSensorTransmittedPulses() const {
	return heightSensorTransmittedPulses;
}

void Flash::setK_hGain(float gain) {
	PRINT("%f\n", gain);
	if(gain != K_hGain) {
		K_hGain = gain;
		writeToFile(K_H_GAIN_FILENAME, gain);
	}
}
void Flash::setK_thetaGain(float gain) {
	if(gain != K_thetaGain) {
		K_thetaGain = gain;
		writeToFile(K_THETA_GAIN_FILENAME, gain);
	}
}
void Flash::setK_qGain(float gain) {
	if(gain != K_qGain) {
		K_qGain = gain;
		writeToFile(K_Q_GAIN_FILENAME, gain);
	}
}
void Flash::setK_vGain(float gain){
	if(gain != K_vGain ){
		K_vGain = gain;
		writeToFile(K_V_GAIN_FILENAME, gain);
	}
}
void Flash::setBaseGain(uint32_t gain){
	if(gain != this->baseGain){
		this->baseGain = gain;
		writeToFile(BASE_GAIN_FILENAME, gain);
	}
}
void Flash::setKal_x(float kal){
	if(kal != kal_x){
		kal_x = kal;
		writeToFile(KAL_X_FILENAME, kal);
	}
}
void Flash::setKal_h(float kal){
	if(kal != kal_h){
		kal_h = kal;
		writeToFile(KAL_H_FILENAME, kal);
	}
}
void Flash::setMaxAngle(uint32_t maxAngle) {
	if(maxAngle != this->maxAngle) {
		this->maxAngle = maxAngle;
		writeToFile(MAX_ANGLE_FILENAME, maxAngle);
	}
}
void Flash::setNeutralAngle(uint32_t neutralAngle){//void Flash::setNeutralAngle(float neutralAngle) {
	if(neutralAngle != this->neutralAngle) {
		this->neutralAngle = neutralAngle;
		writeToFile(NEUTRAL_ANGLE_FILENAME, neutralAngle);
	}
}
void Flash::setHeightSensorNumberOfSamplesToBeAveraged(uint8_t samples) {
	if(samples != heightSensorNumberOfSamplesToBeAveraged) {
		heightSensorNumberOfSamplesToBeAveraged = samples;
		writeToFile(HEIGHT_SENSOR_NUMBER_OF_SAMPLES_TO_BE_AVERAGED_FILENAME, samples);
	}
}
void Flash::setHeightSensorInterval(uint32_t interval) {
	if(interval != heightSensorInterval) {
		heightSensorInterval = interval;
		writeToFile(HEIGHT_SENSOR_INTERVAL_FILENAME, interval);
	}
}
void Flash::setHeightSensorTransmittedPulses(uint16_t pulses) {
	if(pulses != heightSensorTransmittedPulses) {
		heightSensorTransmittedPulses = pulses;
		writeToFile(HEIGHT_SENSOR_TRANSMITTED_PULSES_FILENAME, pulses);
	}
}
void Flash::setActuatorAcceleration(uint32_t acceleration) {
	if(acceleration != actuatorAcceleration) {
		actuatorAcceleration = acceleration;
		writeToFile(ACTUATOR_ACCELERATION_FILENAME, acceleration);
	}
}
void Flash::setActuatorVelocity(uint32_t velocity) {
	if(velocity != actuatorVelocity) {
		actuatorVelocity = velocity;
		writeToFile(ACTUATOR_VELOCITY_FILENAME, velocity);
	}
}

void Flash::setActuatorMaxFollowingError(uint32_t maxFollowingError) {
	if(maxFollowingError != actuatorMaxFollowingError) {
		actuatorMaxFollowingError = maxFollowingError;
		writeToFile(ACTUATOR_MAX_FOLLOWING_ERROR_FILENAME, maxFollowingError);
	}
}

void Flash::parseCanMessage(CANMessage msg) {
	uint32_t id = msg.id;
	//PRINT("Parsing\n");
	//PRINT("ID: %d\n", id);
	if (id == K_H_GAIN_ID || id == K_THETA_GAIN_ID || id == K_Q_GAIN_ID || id == K_V_GAIN_ID || id == FRONT_NEUTRAL_ANGLE_ID ||  id == REAR_NEUTRAL_ANGLE_ID || id == REAR_MAX_ANGLE_ID || id == KAL_X_ID || id == KAL_H_ID) {
		float_to_4_bytes_t gain;
		//PRINT("%x %x %x %x\t", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
		//gain.c[3] = msg.data[3]; gain.c[2] = msg.data[2]; gain.c[1] = msg.data[1]; gain.c[0]=msg.data[0];
		memcpy(&gain, msg.data, 4);
		//PRINT("%x %x %x %x\t", gain.c[0], gain.c[1], gain.c[2], gain.c[3]);
		//PRINT("%f\n", gain.f);
		switch (id) {
			case K_H_GAIN_ID:				setK_hGain(gain.f);			break;
			case K_THETA_GAIN_ID:			setK_thetaGain(gain.f); 	break;
			case K_Q_GAIN_ID:				setK_qGain(gain.f);			break;
			case K_V_GAIN_ID:				setK_vGain(gain.f); 		break;
			case KAL_X_ID:					setKal_x(gain.f);			break;
			case KAL_H_ID:					setKal_h(gain.f);			break;
			//case FRONT_NEUTRAL_ANGLE_ID:	setNeutralAngle(gain.f); 	break;
			//case FRONT_MAX_ANGLE_ID:		setMaxAngle(gain.f); 		break;
			case REAR_NEUTRAL_ANGLE_ID:		setNeutralAngle(gain.f); 	break;
			case REAR_MAX_ANGLE_ID:			setMaxAngle(gain.f);		break;
		}
	} else if(id == HEIGHT_SENSOR_NUMBER_OF_SAMPLES_TO_BE_AVERAGED_ID) {
		setHeightSensorNumberOfSamplesToBeAveraged(msg.data[0]);
	} else if(id == HEIGHT_SENSOR_TRANSMITTED_PULSES_ID) {
		uint16_t data = toUint16(msg.data[1], msg.data[0]);
		setHeightSensorTransmittedPulses(data);
	} else if(id == FRONT_MAX_ANGLE_ID || id == BASE_GAIN_ID || id == FRONT_NEUTRAL_ANGLE_ID|| id == HEIGHT_SENSOR_INTERVAL_ID || id == FRONT_ACTUATOR_ACCELERATION_ID || id == FRONT_ACTUATOR_VELOCITY_ID || id == FRONT_ACTUATOR_MAX_FOLLOWING_ERROR_ID || id == REAR_ACTUATOR_ACCELERATION_ID || id == REAR_ACTUATOR_VELOCITY_ID || id == REAR_ACTUATOR_MAX_FOLLOWING_ERROR_ID) {
		uint32_t data = toUint32(msg.data[3], msg.data[2], msg.data[1], msg.data[0]);
		switch (id) {
			case HEIGHT_SENSOR_INTERVAL_ID: 				setHeightSensorInterval(data); 		break;
			case FRONT_ACTUATOR_ACCELERATION_ID: 			setActuatorAcceleration(data); 		break;
			case FRONT_ACTUATOR_VELOCITY_ID: 				setActuatorVelocity(data); 			break;
			case FRONT_ACTUATOR_MAX_FOLLOWING_ERROR_ID: 	setActuatorMaxFollowingError(data); break;
			case REAR_ACTUATOR_ACCELERATION_ID:				setActuatorAcceleration(data);		break;
			case REAR_ACTUATOR_VELOCITY_ID:					setActuatorVelocity(data);			break;
			case REAR_ACTUATOR_MAX_FOLLOWING_ERROR_ID:		setActuatorMaxFollowingError(data);	break;
			//temporary
			case FRONT_NEUTRAL_ANGLE_ID:					setNeutralAngle(data); 	break;
			case FRONT_MAX_ANGLE_ID:						PRINT("NENENENENENENE\n\n"); setMaxAngle(data); break;
			case BASE_GAIN_ID:								PRINT("JAJAJAJAJAJA\n\n"); setBaseGain(data); break;
		}
	}

}

template<typename writeToFileType>
void Flash::writeToFile(std::string filename, writeToFileType value) {
	FILE *fp = fopen(filename.c_str(), "w");
	if(fp != NULL) {
		fwrite(&value, sizeof(value), 1, fp);
		fclose(fp);
	}
}

template<typename readFromFileType>
readFromFileType Flash::readFromFile(std::string filename) {
	readFromFileType value = 0;
	FILE *fp = fopen(filename.c_str(), "r");
	if(fp != NULL) {
		fread(&value, sizeof(value), 1, fp);
		fclose(fp);
	}
	return value;
}


void Flash::printValues() {
	PRINT("K_h: %f\n", K_hGain);
	PRINT("K_theta: %f\n", K_thetaGain);
	PRINT("K_q: %f\n", K_qGain);
	PRINT("K_v: %f\n", K_vGain);
	PRINT("kal_x: %f\n", kal_x);
	PRINT("kal_h: %f\n", kal_h);
	//PRINT("NOM AVERAGED: %d\n", heightSensorNumberOfSamplesToBeAveraged);
	//PRINT("INTERVAL: %d\n", heightSensorInterval);
	//PRINT("PULSES: %d\n", heightSensorTransmittedPulses);
	PRINT("ACCELERATION: %d\n", actuatorAcceleration);
	PRINT("VELOCITY: %d\n", actuatorVelocity);
	PRINT("MAX FOLLOWING ERROR: %d\n", actuatorMaxFollowingError);
	PRINT("MAX ANGLE: %i\n", maxAngle);
	PRINT("NEUTRAL ANGLE: %i", neutralAngle);
	PRINT("BASE GAIN: %i\n\n", baseGain);
}

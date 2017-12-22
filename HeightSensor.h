/*
 * HeightSensor.h
 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#ifndef HEIGHTSENSOR_H_
#define HEIGHTSENSOR_H_

#include "mbed.h"
#include "utilities/utilities.h"
#include "MODSERIAL/MODSERIAL.h"
#include "config.h"
#include "rtos.h"
#include "CanManager.h"

#define ASCII_STREAM 1
// When the height sensors has an measuring error, it outputs 0 as height
#define HEIGHT_SENSOR_ERROR_OUTPUT 0

// Modbus message to request sensor's settings
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_ADDRESS			0x01
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_FUNC_CODE			0x03
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_START_ADDR_MSB	0x00
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_START_ADDR_LSB	0x02
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_REG_COUNT_MSB		0x00
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_REG_COUNT_LSB		0x05
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_CRC_MSB			0x09
#define HEIGHT_MESSAGE_REQUEST_SETTINGS_CRC_LSB			0x24

// Modbus message to request height
#define HEIGHT_MESSAGE_REQUEST_ADDRESS			0x01
#define HEIGHT_MESSAGE_REQUEST_FUNC_CODE		0x03
#define HEIGHT_MESSAGE_REQUEST_START_ADDR_MSB	0x02
#define HEIGHT_MESSAGE_REQUEST_START_ADDR_LSB	0x08
#define HEIGHT_MESSAGE_REQUEST_REG_COUNT_MSB	0x00
#define HEIGHT_MESSAGE_REQUEST_REG_COUNT_LSB	0x01
#define HEIGHT_MESSAGE_REQUEST_CRC_MSB			0x70
#define HEIGHT_MESSAGE_REQUEST_CRC_LSB			0x04

// Modbus message to set filters
#define HEIGHT_SENSOR_SET_FILTER_ADDRESS			0x01
#define HEIGHT_SENSOR_SET_FILTER_FUNC_CODE			0x10
#define HEIGHT_SENSOR_SET_FILTER_START_ADDR_MSB		0x00
#define HEIGHT_SENSOR_SET_FILTER_START_ADDR_LSB		0x02
#define HEIGHT_SENSOR_SET_FILTER_REG_COUNT_MSB		0x00
#define HEIGHT_SENSOR_SET_FILTER_REG_COUNT_LSB		0x01
#define HEIGHT_SENSOR_SET_FILTER_TOTAL_BYTE_COUNT	0x02

// Modbus message to set number of samples to be averaged
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_ADDRESS			0x01
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_FUNC_CODE		0x10
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_START_ADDR_MSB	0x00
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_START_ADDR_LSB	0x03
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_REG_COUNT_MSB	0x00
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_REG_COUNT_LSB	0x01
#define HEIGHT_SENSOR_SET_NO_OF_SAMPLES_TO_BE_AVERAGED_TOTAL_BYTE_COUNT	0x02

// Modbus message to set interval
#define HEIGHT_SENSOR_SET_INTERVAL_ADDRESS 			0x01
#define HEIGHT_SENSOR_SET_INTERVAL_FUNC_CODE 		0x10
#define HEIGHT_SENSOR_SET_INTERVAL_START_ADDR_MSB 	0x00
#define HEIGHT_SENSOR_SET_INTERVAL_START_ADDR_LSB 	0x04
#define HEIGHT_SENSOR_SET_INTERVAL_REG_COUNT_MSB 	0x00
#define HEIGHT_SENSOR_SET_INTERVAL_REG_COUNT_LSB 	0x02
#define HEIGHT_SENSOR_SET_INTERVAL_TOTAL_BYTE_COUNT 0x04

// Modbus message to set transmitted pulses
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_ADDRESS			0x01
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_FUNC_CODE			0x10
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_START_ADDR_MSB		0x00
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_START_ADDR_LSB		0x06
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_REG_COUNT_MSB		0x00
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_REG_COUNT_LSB		0x01
#define HEIGHT_SENSOR_SET_TRANSMITTED_PULSES_TOTAL_BYTE_COUNT	0x02

// Modbus height request response
#define HEIGHT_MESSAGE_RESPONSE_ADDRESS		0
#define HEIGHT_MESSAGE_RESPONSE_FUNCTION	1
#define HEIGHT_MESSAGE_RESPONSE_BYTE_COUNT	2
#define HEIGHT_MESSAGE_RESPONSE_VALUE_MSB	3
#define HEIGHT_MESSAGE_RESPONSE_VALUE_LSB	4
#define HEIGHT_MESSAGE_RESPONSE_CRC_MSB		6
#define HEIGHT_MESSAGE_RESPONSE_CRC_lSB		5

// Modbus settings request response
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_ADDRESS							0
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_FUNC_CODE							1
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_BYTE_COUNT							2
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_FILTERS_MSB						3
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_FILTERS_LSB						4
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_NUMBER_OF_SAMPLES_TO_BE_AVERAGED 	6
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_INTERVAL_MSW_MSB					7
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_INTERVAL_MSW_LSB					8
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_INTERVAL_LSW_MSB					9
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_INTERVAL_LSW_LSB					10
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_PULSES_MSB							11
#define HEIGHT_MESSAGE_SETTINGS_RESPONSE_PULSES_LSB							12

// Height sensor timeout time, when waiting for modbus response
#define HEIGHT_SENSOR_CLEAR_BUFFER_TIMEOUT			100
#define HEIGHT_SENSOR_RESPONSE_TIMEOUT				50
#define HEIGHT_SENSOR_ASCII_TIMEOUT					15

// Message buffer lengths
#define	 HEIGHT_SENSOR_GET_HEIGHT_BUFFER_LENGTH		7
#define	 HEIGHT_SENSOR_GET_SETTINGS_BUFFER_LENGTH	15
#define HEIGHT_SENSOR_ASCII_BUFFER_LENGTH			6

// Defaults
#define HEIGHT_SENSOR_DEFAULT_FILTERS				0b0000000100000000


namespace sbt {

class HeightSensor {
private:
	uint16_t filters;
	uint8_t numberOfSamplesToBeAveraged;
	uint32_t interval;
	uint16_t transmittedPulses;
	float senixHeightOffset;

	Timer 		*statisticsTimer;
	float 		stat_pps;  // pulses per second
	uint16_t 	stat_eptp; // error per 10 pulses
	float 		stat_stdev;// stdev measurements
	uint16_t 	stat_stdev_count; // amount of stdev measurements
	float 		stat_high; // highest distance
	float 		stat_low;  // lowest distance
	uint16_t	stat_fptp; // Filtered pulse count per 10 pulses
	uint16_t	stat_tptp; // Senix pulse timeout per 10 pulses

	uint8_t nodeID;

	void statisticsReset();
	MODSERIAL* s;
	Timer timer;
	Timer ageTimer;
	float height;
	uint8_t buffer[8];
	HeightSensorException waitForReply(uint8_t buffer[], int length);
public:
	Semaphore *sem;
	Mutex *mutex;
	Mutex *busyMutex;
	HeightSensor(MODSERIAL* s,uint8_t);
	Thread *processThread;
	float getHeight();
	int getAge();
	bool idle;
	void CANReport(CanManager *, HeightSensor *);
	void CANReportStatistics(CanManager *);

	uint16_t getFilters() const;
	uint32_t getInterval() const;
	uint8_t getNumberOfSamplesToBeAveraged() const;
	uint16_t getTransmittedPulses() const;
	void thread (void const *TID);

	void flushStatistics(CanManager *canManager);

	uint16_t CRC16(uint8_t *nData, uint16_t wLength);
	void requestSettings();
	void requestHeight();
		HeightSensorException read(float *height);
	HeightSensorException setFilters(uint16_t filters);
	HeightSensorException setNumberOfSamplesToBeAveraged(uint8_t numberOfSamples);
	HeightSensorException setInterval(uint32_t interval);
	HeightSensorException setTransmittedPulses(uint16_t pulses);
	HeightSensorException clearBuffer(int byteCount);
	HeightSensorException getRequestedSettings();
	HeightSensorException getRequestedHeight(uint16_t* reply);
	HeightSensorException parse(float *height);
	void heightSensorISR();
	HeightSensorException ASCIIStreamRead();
	HeightSensorException ASCIIStreamParse();

	void printValues();
};

} /* namespace sbt */

#endif /* HEIGHTSENSOR_H_ */

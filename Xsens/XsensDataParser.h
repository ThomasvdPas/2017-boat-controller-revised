/*
 * XsensDataParser.h
 *
 *  Created on: 13 feb. 2014
 *      Author: boris
 */
#include <inttypes.h>

#ifndef XSENSDATAPARSER_H_
#define XSENSDATAPARSER_H_

#define XSENS_ROLL_BIAS_CORRECTION		0.75f

namespace sbt {// TODO do we use namespaces?
typedef struct { // TODO Move this struct definition to its own file
	struct {
		uint32_t ns;
		uint16_t year;
		uint8_t month;
		uint8_t day;
		uint8_t hour;
		uint8_t minute;
		uint8_t second;
		uint8_t flags;
	} time;
	struct {
		uint16_t count;
		uint16_t prev;
	} packetCounter;
	struct {
		float roll;
		float pitch;
		float yaw;
	} rotation;
	struct {
		float x;
		float y;
		float z;
	} acceleration;
	struct {
		float lat;
		float lon;
	} latlon;
	float altitude;
	float velocity;
	float velocity_xy;
	struct {
		float x;
		float y;
		float z;
	} rot;
	struct {
		int pressure;
	} baro;
	union {
		struct {
			uint32_t selftest : 1;
			uint32_t filterValid : 1;
			uint32_t gpsFix : 1;
			uint32_t noRotationUpdateStatus : 2;
			uint32_t timestampGpsSynced : 1;
			uint32_t timestampClockSynced : 1;
			uint32_t onOff : 1;
			uint32_t clipflagAccX : 1;
			uint32_t clipflagAccY : 1;
			uint32_t clipflagAccZ : 1;
			uint32_t clipflagGyrX : 1;
			uint32_t clipflagGyrY : 1;
			uint32_t clipflagGyrZ : 1;
			uint32_t clipflagMagX : 1;
			uint32_t clipflagMagY : 1;
			uint32_t clipflagMagZ : 1;
			uint32_t _reserved1 : 2;
			uint32_t clippingIndication : 1;
			uint32_t _reserved2 : 1;
			uint32_t syncInMarker : 1;
			uint32_t syncOutmarker : 1;
			uint32_t filterMode : 3;
			uint32_t _reserved3 : 6;
		};
		uint32_t bytes;
	} statusWord;
} xsens_data_t;


class XsensDataParser {
private:

	xsens_data_t* data;

	enum DataIds {
		UTC_TIME = 0x1010,
		PACKET_COUNTER = 0x1020,
		EULER_ANGLES = 0x2030,
		ACCELERATION = 0x4030,
		STATUS_WORD = 0xE020,
		ALTITUDE = 0x5020,
		GNSSPVT = 0x7010,
		VELOCITYXYZ = 0xD010,
		ROT = 0x8020,
		BAROPRESSURE = 0x3010
	};

	void parseUtcTime(uint8_t message[], int offset);
	void parseEulerAngles(uint8_t message[], int offset);
	void parseAcceleration(uint8_t message[], int offset);
	void parseStatusWord(uint8_t message[], int offset);
	void parseAltitude(uint8_t message[], int offset);
	void parseRot(uint8_t message[], int offset);
	void parseBaroPressure(uint8_t[], int);
	void parsePacketCounter(uint8_t[], int);
	void parseGnssPvt(uint8_t[], int);
	void parseVelocityXYZ(uint8_t[], int);

public:
	XsensDataParser(xsens_data_t*);
	virtual ~XsensDataParser();
	void parse(uint8_t data[], int length);
};
}
#endif /* XSENSDATAPARSER_H_ */

/*
 * XsensDataParser.cpp
 *
 *  Created on: 13 feb. 2014
 *      Author: boris
 */

#include "mbed.h"
#include "../utilities/utilities.h"
#include "Xsens/XsensDataParser.h"
#include "rtos.h"

 extern Serial pc;
extern Mutex pcMutex;

namespace sbt {

XsensDataParser::XsensDataParser(xsens_data_t* data){
	this->data = data;
}

void XsensDataParser::parse(uint8_t d[], int l) {
	int c = 4;
	uint16_t len = d[3];
	// extended length
	if(d[3] == 0xFF) {
		// concatenate the two extended length bytes
		len = toUint16(d[4], d[5]);
		c = 6;
	}
	while(c<len && c<l) {
		uint16_t dataId = toUint16(d[c], d[c+1]);
		c+=2;
		uint8_t dataLen = d[c++];
		switch(dataId) {
			case UTC_TIME: parseUtcTime(d, c); break;
			case PACKET_COUNTER: parsePacketCounter(d, c); break;
			case EULER_ANGLES: parseEulerAngles(d, c); break;
			case ACCELERATION: parseAcceleration(d, c); break;
			case STATUS_WORD: parseStatusWord(d, c); break;
			case GNSSPVT:
				parseGnssPvt(d, c);
				break;
			case VELOCITYXYZ:
				parseVelocityXYZ(d, c);
				break;
//			case ALTITUDE: parseAltitude(d, c); break;
			case ROT: parseRot(d, c); break;
			case BAROPRESSURE: parseBaroPressure(d, c); break;
		}
		c += dataLen;
	}
}

void XsensDataParser::parseGnssPvt(uint8_t mess[], int o) {
//	for (int i =o; i < 76+o; i+=4) {
//		PRINT(" %d -> %2x %2x %2x %2x \t %f\n",i -o, mess[i], mess[i+1], mess[i+2], mess[i+3], toUint32(mess[i+0], mess[i+1], mess[i+2], mess[i+3])/10000000.0f);
//	}
//	PRINT(" lat: %f\n",toFloat(mess[o+31], mess[o+30], mess[o+29], mess[o+28]))

	data->latlon.lat = toUint32(mess[o+24], mess[o+25], mess[o+26], mess[o+27])/10000000.0f;
	data->latlon.lon = toUint32(mess[o+28], mess[o+29], mess[o+30], mess[o+31])/10000000.0f;
	data->altitude = toUint32(mess[o+36], mess[o+37], mess[o+38], mess[o+39]);
	data->velocity = toUint32(mess[o+60], mess[o+61], mess[o+62], mess[o+63])/1000.0f;

//	PRINT("lat/lon/alt/v: %f \t %f \t %f \t %f\n",data->latlon.lat,data->latlon.lon, data->altitude ,data->velocity);
}

void XsensDataParser::parseUtcTime(uint8_t mess[], int o) {
	data->time.ns = toUint32(mess[o+3], mess[o+2], mess[o+1], mess[o]);
	data->time.year = toUint16(mess[o+4], mess[o+5]);
	data->time.month = mess[o+6];
	data->time.day = mess[o+7];
	data->time.hour = mess[o+8];
	data->time.minute = mess[o+9];
	data->time.second = mess[o+10];
	data->time.flags = mess[o+11];
}

void XsensDataParser::parseEulerAngles(uint8_t mess[], int o) {
	data->rotation.roll = toFloat(mess[o+3], mess[o+2], mess[o+1], mess[o]);
//	PRINT(" pitch -> %2x %2x %2x %2x\n",mess[o+7], mess[o+6], mess[o+5], mess[o+4]);
	data->rotation.pitch = toFloat(mess[o+7], mess[o+6], mess[o+5], mess[o+4]);
	data->rotation.yaw = toFloat(mess[o+11], mess[o+10], mess[o+9], mess[o+8]);
}

void XsensDataParser::parseAcceleration(uint8_t mess[], int o) {
	data->acceleration.x = toFloat(mess[o+3], mess[o+2], mess[o+1], mess[o]);
	data->acceleration.y = toFloat(mess[o+7], mess[o+6], mess[o+5], mess[o+4]);
	data->acceleration.z = toFloat(mess[o+11], mess[o+10], mess[o+9], mess[o+8]);
}
//
//void XsensDataParser::parseAltitude(uint8_t mess[], int o) {
//	data->altitude = toFloat(mess[o+3], mess[o+2], mess[o+1], mess[o]);
//}

void XsensDataParser::parseRot(uint8_t mess[], int o) {
	data->rot.x = toFloat(mess[o+3], mess[o+2], mess[o+1], mess[o]);
	data->rot.y = toFloat(mess[o+7], mess[o+6], mess[o+5], mess[o+4]);
	data->rot.z = toFloat(mess[o+11], mess[o+10], mess[o+9], mess[o+8]);
}

void XsensDataParser::parseStatusWord(uint8_t mess[], int o) {
	data->statusWord.bytes = toUint32(mess[o+3], mess[o+2], mess[o+1], mess[o]);
}


void XsensDataParser::parseBaroPressure(uint8_t mess[], int o) {
	data->baro.pressure = toUint32(mess[o], mess[o+1], mess[o+2], mess[o+3]);
}

void XsensDataParser::parsePacketCounter(uint8_t mess[], int o) {
	data->packetCounter.prev = data->packetCounter.count;
	data->packetCounter.count = toUint16(mess[o], mess[o+1]);
}

void XsensDataParser::parseVelocityXYZ(uint8_t mess[], int o) {
	float velX = toFloat(mess[o+3], mess[o+2], mess[o+1], mess[o]);
	float velY = toFloat(mess[o+7], mess[o+6], mess[o+5], mess[o+4]);
	data->velocity_xy = sqrt(velX * velX + velY * velY);
}

XsensDataParser::~XsensDataParser() {

}

}

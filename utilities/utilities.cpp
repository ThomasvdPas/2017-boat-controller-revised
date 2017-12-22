/*
 * utilities.cpp

 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#include <algorithm>
#include "mbed.h"
#include "utilities.h"
#include "../MODSERIAL/MODSERIAL.h"


void setBit(uint16_t* byte, uint8_t nbit){
	//nbit is the bit to set.
	*byte |= (1 << nbit);
}

void clearBit(uint16_t* byte, uint8_t nbit){
	*byte &= ~(1 << nbit);
}

bool checkBit(uint16_t byte, uint8_t nbit){
	if((byte & 1<<nbit)==1<<nbit){
		return true;
	} else{
		return false;
	}
}

float toRadians(float degrees) {
	return degrees * PI / 180;
}

/*
int32_t toQc(float angle) {
	return angle * ANGLE_TO_QC;
} */

float toFloat(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	float f;
	uint8_t arr[] = {a,b,c,d};
	memcpy(&f, arr, 4);
	return f;
}

uint16_t toUint16(uint8_t a, uint8_t b) {
	return a << 8 | b;
}
uint32_t toUint32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	return a << 24 | b << 16 | c << 8 | d;
}
uint32_t toUint32(uint16_t a, uint16_t b) {
	return a << 16 | b;
}

/*
int32_t getNeutralWingQc(float neutralWingAngle) {
	return (neutralWingAngle-HOMING_ANGLE) * ANGLE_TO_QC;
}
*/

/*
int32_t absoluteAngleToQc(float absoluteAngle) {
	return ((float) ((float)absoluteAngle-(float)HOMING_ANGLE)) * ANGLE_TO_QC;
}
*/

float rawHeightToMeters(uint16_t rawHeight) {
	return rawHeight * 0.003384 * 0.0254;
}

bool in_array(const std::string &value, const std::vector<std::string> &array)
{
    return std::find(std::begin(array), array.end(), value) != array.end();
}

/*
 * utilities.h
 *
 *  Created on: Mar 11, 2014
 *      Author: boris
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <string>
#include <vector>
#include "util_debug.h"
#include "bit_operations.h"
#include "exceptions.h"
#include "../MODSERIAL/MODSERIAL.h"
#include "arm_math.h"

#define max(x,y)	( x>y ? x : y)
#define min(x,y)	( x<y ? x : y)
#define abs(x)	(max(x,-x)-min(x,-x))

//#define PI 							3.14159265
//#define MIN_ANGLE						-15

void setBit(uint16_t* byte, uint8_t nbit);
void clearBit(uint16_t* byte, uint8_t nbit);
bool checkBit(uint16_t byte, uint8_t nbit);

float toRadians(float degrees);
int32_t toQc(float angle);
float rawHeightToMeters(uint16_t rawHeight);

//const float PI = 3.1415927;

float toFloat(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint16_t toUint16(uint8_t a, uint8_t b);
uint32_t toUint32(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint32_t toUint32(uint16_t a, uint16_t b);

void modserialPutArray(MODSERIAL* s, char array[], int length);
void modserialPutArray(MODSERIAL* s, uint8_t array[], int length);

int32_t getNeutralWingQc(float neutralWingAngle);
int32_t absoluteAngleToQc(float absoluteAngle);

bool in_array(const string &value, const std::vector<string> &array);

typedef union {
	float f[2];
	char c[8];
} two_floats_to_8_bytes_t;

typedef union {
	float f;
	char c[4];
} float_to_4_bytes_t;

typedef union {
	int8_t i[2];
	char c[2];
} two_int8_to_2_bytes_t;

typedef union {
	int8_t i[6];
	char c[6];
} six_int8_to_6_bytes_t;

typedef union {
	int8_t i[8];
	char c[8];
} eight_int8_to_8_bytes_t;

typedef union {
	int8_t i[5];
	char c[5];
} five_int8_to_5_bytes_t;

typedef union {
	int32_t i;
	char c[4];
} int32_to_4_bytes_t;

typedef union {
	int32_t i[2];
	char c[8];
} two_int32_to_8_bytes_t;

typedef union {
	uint32_t i;
	char c[4];
} uint32_to_4_bytes_t;

typedef union {
	uint16_t i;
	char c[2];
} uint16_to_2_bytes_t;


typedef union {
	int16_t i[2];
	char c[4];
} two_int16_to_4_bytes_t;


typedef union {
	struct {
		uint16_t year;
		uint8_t month;
		uint8_t day;
		uint8_t hour;
		uint8_t minute;
		uint8_t second;
		uint8_t flags;
	};
	char c[8];
} time_to_8_bytes_t;



#endif /* UTILITIES_H_ */

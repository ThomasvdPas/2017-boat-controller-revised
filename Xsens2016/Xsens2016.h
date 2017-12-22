/*
 * Xsens.h
 *
 *  Created on: 13 feb. 2014
 *      Author: boris
 */
#include <inttypes.h>
#include "mbed.h"
#include "utilities/utilities.h"
#include "Xsens/XsensDataParser.h"
#include "MODSERIAL/MODSERIAL.h"

#ifndef XSENS_H_
#define XSENS_H_

#define USE_TIMEOUT 1

#define XSENS_DEFAULT_TIMEOUT 			2900
#define XSENS_WAKE_UP_TIMEOUT			3000
#define XSENS_RESET_TIMEOUT				2000

#define XSENS_STACK_SIZE					9500
#define XSENS_MAXDATALEN					8192
#define XSENS_MESSAGE_BUFFER_LENGTH 	8192
#define XSENS_MSG_MTDATA2_ID				0x36
#define XSENS_MSG_ERROR_ID				0x42

#define XSENS_PREAMBLE					0xFA
#define XSENS_BID							0xFF

// request message
#define XSENS_MESSAGE_REQUEST_PREAMBLE 	0xFA
#define XSENS_MESSAGE_REQUEST_BID 		0xFF
#define XSENS_MESSAGE_REQUEST_MID 		0x34
#define XSENS_MESSAGE_REQUEST_LENGTH 	0x00
#define XSENS_MESSAGE_REQUEST_CHECKSUM 	0xCD

// reset message
#define XSENS_RESET_PREAMBLE				0xFA
#define XSENS_RESET_BID					0xFF
#define XSENS_RESET_MID					0x40
#define XSENS_RESET_LENGTH				0x00
#define XSENS_RESET_CHECKSUM				0xC1

// list of MID's
#define XSENS_GOTOCONFIG_MID				0x30
#define XSENS_GOTOCONFIGACK_MID			0x31
#define XSENS_GOTOMEASUREMENT_MID		0x10
#define XSENS_GOTOMEASUREMENTACK_MID	0x11
#define XSENS_REQCONFIG_MID				0x0C
#define XSENS_SETCONFIG_MID				0x0D
#define XSENS_SETBAUDRATE_MID			0x24
#define XSENS_SETBAUDRATEACK_MID			0x25


// message
const int XSENS_HEADER_SHORT =				4; // without ext length field
const int XSENS_HEADER_LONG = 				6; // with ext length field
const int XSENS_HEADER_SHORT_CS = 			(XSENS_HEADER_SHORT+1); // including CS
const int XSENS_HEADER_LONG_CS =			(XSENS_HEADER_LONG+1);
const int XSENS_DAT_LEN_TIME = 			12;
const int XSENS_DAT_LEN_ROTATION = 		36;
const int XSENS_DAT_LEN_ACCELERATION = 	12;
const int XSENS_DAT_LEN_LATLON = 			8;
const int XSENS_DAT_LEN_VELOCITY = 		12;
const int XSENS_DAT_LEN_STATUSWORD = 		4;
const int XSENS_MESSAGE_TOTAL_LEN =		(XSENS_HEADER_LONG_CS + XSENS_DAT_LEN_TIME + XSENS_DAT_LEN_ROTATION + XSENS_DAT_LEN_ACCELERATION + XSENS_DAT_LEN_LATLON + XSENS_DAT_LEN_VELOCITY + XSENS_DAT_LEN_STATUSWORD);

// xsens message
#define XSENS_MESSAGE_PREAMBLE				0xFA
#define XSENS_MESSAGE_BID					0xFF
#define XSENS_MESSAGE_MTDATA2				0x36
#define XSENS_MESSAGE_ERROR					0x42
#define XSENS_MESSAGE_USE_EXTENDED_LENGTH	0xFF
#define XSENS_

// xsens message positions
#define XSENS_MESSAGE_POSITION_PREAMBLE	0
#define XSENS_MESSAGE_POSITION_BID		1
#define XSENS_MESSAGE_POSITION_MID		2

const int XSENS_READ_BUFF_SIZE =		XSENS_MESSAGE_TOTAL_LEN * 16;

class Xsens {
private:
	MODSERIAL* _serial;
	void reset();
public:
	Xsens(MODSERIAL*);
	void requestMessage();
	XsensException xsensReset();
	XsensException waitForWakeUp();
	XsensException bufferDataMessage(uint8_t data[], int* dataLength);
};

#endif /* XSENS_H_ */

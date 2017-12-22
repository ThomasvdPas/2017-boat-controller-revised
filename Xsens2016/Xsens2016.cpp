/*
 * Xsens.cpp

 *
 *  Created on: 25 feb. 2016
 *      Author: Bart & Boris
 */
#include <stdlib.h>
#include <stdio.h>
#include "mbed.h"
#include "utilities/utilities.h"
#include "Xsens2016/Xsens.h"
#include "MODSERIAL/MODSERIAL.h"

 extern Serial pc;
extern Mutex pcMutex;

Xsens::Xsens(MODSERIAL* s){
	_serial = s;

void Xsens::requestMessage() {
	//FA FF 34 00 CD
	_serial->putc(XSENS_MESSAGE_REQUEST_PREAMBLE);
	_serial->putc(XSENS_MESSAGE_REQUEST_BID);
	_serial->putc(XSENS_MESSAGE_REQUEST_MID);
	_serial->putc(XSENS_MESSAGE_REQUEST_LENGTH);
	_serial->putc(XSENS_MESSAGE_REQUEST_CHECKSUM);
}

XsensException Xsens::xsensReset() {
	uint8_t buffer[5];
	uint8_t bufferCount=0;
	Timer timer;

	//FA FF 40 00 C1
	_serial->putc(XSENS_RESET_PREAMBLE);
	_serial->putc(XSENS_RESET_BID);
	_serial->putc(XSENS_RESET_MID);
	_serial->putc(XSENS_RESET_LENGTH);
	_serial->putc(XSENS_RESET_CHECKSUM);

	timer.start();
	while(bufferCount<5) {
		if(USE_TIMEOUT == 1 && timer.read_ms() >= XSENS_RESET_TIMEOUT) {
			return XsensException::timeout;
		}
		if(_serial->readable()) {
			buffer[bufferCount] = (uint8_t) _serial->getc();
			bufferCount++;
		}
	}
	if(
		buffer[0] != 0xFA ||
		buffer[1] != 0xFF ||
		buffer[2] != 0x41 ||
		buffer[3] != 0x00 ||
		buffer[4] != 0xC0
	) {
		return XsensException::wrongMessageHeader;
	}
	return XsensException::noException;
}

XsensException Xsens::waitForWakeUp() {
	uint8_t buffer[5];
	uint8_t bufferCount=0;
	Timer timer;
	timer.start();
	while(bufferCount<5) {
		if(USE_TIMEOUT && (timer.read_ms() >= XSENS_WAKE_UP_TIMEOUT)) {
			return XsensException::timeout;
		}
		if(_serial->readable()) {
			buffer[bufferCount] = (uint8_t) _serial->getc();
			bufferCount++;
		}
	}
	if(
		buffer[0] != 0xFA ||
		buffer[1] != 0xFF ||
		buffer[2] != 0x3E ||
		buffer[3] != 0x00 ||
		buffer[4] != 0xC3
	) {
		return XsensException::wrongMessageHeader;
	}
	return XsensException::noException;
}


XsensException Xsens::bufferDataMessage(uint8_t buffer[], int* bufferCount) {
//This functions fills the buffer
}
	*bufferCount = 0;
	uint8_t headerCsLength = 5;
	uint16_t length = 0;

	bool preambleFound = 0;

	Timer timer;
	timer.start();
	// make sure the length field gets read, and terminate when all message bytes were read
	while(true) {

		if(USE_TIMEOUT == 1 && timer.read_ms() >= XSENS_DEFAULT_TIMEOUT) {
			return XsensException::timeout;
		}
		if(_serial->readable()) {
			uint8_t c = (uint8_t) _serial->getc();

			if(preambleFound == false && c ==XSENS_MESSAGE_PREAMBLE) {
				preambleFound = true;
			}

			if(preambleFound) {
				// check to prevent memory overflow
				if(length > XSENS_MESSAGE_BUFFER_LENGTH) {
					return XsensException::messageTooLarge;
				}
				// length if not extended
				if(*bufferCount == 3 && c != XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length = c;
				}
				// length if extended
				if(*bufferCount == 4 && buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length = c << 8;
					headerCsLength = 6;
				}
				if(*bufferCount == 5 && buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length |= (uint16_t) c;
				}

				// Place into buffer
				buffer[*bufferCount] = c;
				(*bufferCount)++;



				// wait untill message is fully buffered
				if(*bufferCount > 3) {
					if(buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH && *bufferCount > 6 && (*bufferCount - headerCsLength) == length) {
						break;
					} else if(*bufferCount - headerCsLength == length) {
						break;
					}
				}
			}
		}
	}
}


/*
 * Xsens.cpp

 *
 *  Created on: 13 feb. 2014
 *      Author: boris
 */
#include <stdlib.h>
#include <stdio.h>
#include "mbed.h"
#include "rtos.h"
#include "utilities/utilities.h"
#include "utilities/exceptions.h"
#include "Xsens/Xsens.h"
#include "Xsens/XsensDataParser.h"
#include "MODSERIAL/MODSERIAL.h"
#include "config.h"

#ifdef DEBUG
 extern Serial pc;
extern Mutex pcMutex;
#endif

Xsens::Xsens(MODSERIAL* s) : _serial(s) {
	this->_serial->baud(XSENS_BAUD_RATE);
	this->_serial->rxBufferFlush();
	this->xsensData_mutex = new Mutex();
	this->sem = new Semaphore(1);
	this->interpreter = new XsensDataInterpreter(&(this->xsensData));
}

void Xsens::printState() {
	this->interpreter->printState();
}

void Xsens::thread(void const *TID) {

	XsensDataParser xdp(&(this->xsensData));
	XsensException xe;
	XsensErrorParser xep(&xe);

	uint8_t bufferLength = 0;

	int xsenscounter = 0;
	int Xsenserrorcounter = 0;
	int Vcounter = 0;
	int bufferSize = XSENS_MESSAGE_BUFFER_LENGTH;

	Timer XsensTimer;

	PRINT("Xsens_thread started\n");

	while(1<2){
		//Wait for the Rx interrupt to clear the semaphore
		this->sem->wait();
		if (xsenscounter == 0){
			XsensTimer.start();
		}

		uint8_t buffer[XSENS_MESSAGE_BUFFER_LENGTH] = {0};

		XsensException xsensException = receiveMessage(buffer, &bufferLength, bufferSize);

		if(xsensException == XsensException::noException) {
			if(buffer[XSENS_MESSAGE_POSITION_MID] == XSENS_MESSAGE_MTDATA2) {
				xdp.parse(buffer, bufferLength);

				this->xsensData_mutex->lock();
//				theta = - interpreter.getPitch(); //TODO::butter(interpreter.getPitch());
				//PRINT("THETA= %f \t\t", theta);
//				q = - interpreter.getRotY()*180/PI; //TODO::butter(interpreter.getRotY());
				//PRINT("Q= %f \t\t\n", q);
//				V = interpreter.getVelocityBoat();
//				a_z = - interpreter.getAccelerationZ();
//				xsens_mutex.unlock();
				this->xsensData_mutex->unlock();
				//checkGPS(); //check if there is GPSlock.
				this->processThread->signal_set(0x1);



			} else if(buffer[XSENS_MESSAGE_POSITION_MID] == XSENS_MESSAGE_ERROR) {
				Xsenserrorcounter += 0;
				xep.parse(buffer, bufferLength);
				PRINT("%i", xe);
				//writeCanError(CanErrorCategory::xsens, (uint8_t) xe); //TODO write can error
				Xsenserrorcounter++;
			}


		}
		xsenscounter++;

		if (xsenscounter == 4000){
			XsensTimer.stop();
			PRINT("Xsens 4000 cycles dt: %f sec, %i /4000 errors, buffercount %i \n" ,
				  XsensTimer.read(), Xsenserrorcounter, this->_serial->rxBufferGetCount());
			xsenscounter = 0;
			Xsenserrorcounter = 0;
			XsensTimer.reset();
		}

	}

}

void Xsens::requestMessage() {
	//FA FF 34 00 CD
	_serial->putc(XSENS_MESSAGE_REQUEST_PREAMBLE);
	_serial->putc(XSENS_MESSAGE_REQUEST_BID);
	_serial->putc(XSENS_MESSAGE_REQUEST_MID);
	_serial->putc(XSENS_MESSAGE_REQUEST_LENGTH);
	_serial->putc(XSENS_MESSAGE_REQUEST_CHECKSUM);
}

void Xsens::transmitReset() {

	_serial->putc(XSENS_RESET_PREAMBLE);
	_serial->putc(XSENS_RESET_BID);
	_serial->putc(XSENS_RESET_MID);
	_serial->putc(XSENS_RESET_LENGTH);
	_serial->putc(XSENS_RESET_CHECKSUM);
}

XsensException Xsens::xsensReset() {
	const uint16_t size = 5;
	uint8_t buffer[size]; // TODO Hij kan buiten de buffer schrijven!
	uint8_t bufferCount=0;
//	Timer timer;
//	bool preamblefound = 0;

	//FA FF 40 00 C1
//	PRINT("transmitreset");


//	PRINT("wait for reset ack\r\n");
	XsensException err;
	for (int i =0; i < 12; i++) {
		if (i == 0 || i == 6) {
			transmitReset();
		}
		err = receiveMessage(buffer, &bufferCount, size);
		PRINT("\r\nreceived (err: %d) (buffercount: %d)!\r\n", err, bufferCount);
		for (int i = 0; i < size; i++) {
			PRINT("%02x ", buffer[i] & 0xFF);
		}
		if (err == XsensException::noException && (
				buffer[0] == 0xFA &&
				buffer[1] == 0xFF &&
				buffer[2] == 0x3E &&
				buffer[3] == 0x00 &&
				buffer[4] == 0xC3
			)) {
			break;
		}
	}
	if (err != XsensException::noException) {
		return err;
	}

	if(
		buffer[0] != 0xFA ||
		buffer[1] != 0xFF ||
		buffer[2] != 0x3E ||
		buffer[3] != 0x00 ||
		buffer[4] != 0xC3
	) {
		PRINT("reset failed");
		return XsensException::wrongMessageHeader;
	}
	PRINT("reset success");
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

uint8_t Xsens::getchar2(){
	uint8_t c = _serial->getc();
	return  c;
}
XsensException Xsens::getChar(uint8_t buffer[], int bufferCount, uint16_t* dataLength){
	static bool preambleFound = false;
	static uint8_t sum = 0;
	static uint8_t headerCsLength = 5;
	static uint16_t length = 0;

	*dataLength = length;

	Timer timer;
	timer.start();
	while(true) {

		if (USE_TIMEOUT == 1 && timer.read_ms() >= XSENS_DEFAULT_TIMEOUT) {
			PRINT("timeout");
			return XsensException::timeout;
		}
		if (_serial->readable()) {
			uint8_t c = _serial->getc();



			if (preambleFound == false && c == XSENS_MESSAGE_PREAMBLE) {
				preambleFound = true;
			}

			if (preambleFound) {
				// check to prevent memory overflow
				if (length > XSENS_MESSAGE_BUFFER_LENGTH) {
					return XsensException::messageTooLarge;
				}
				// length if not extended
				if (bufferCount == 3 && c != XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length = c;
				}
				// length if extended
				if (bufferCount == 4 && buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length = c << 8;
					headerCsLength = 6;
				}
				if (bufferCount == 5 && buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH) {
					length |= (uint16_t) c;
				}

				buffer[bufferCount] = c;
				// skip preamble in checksum calculation
				if (bufferCount > 0) {
					sum += c;
				}
				(bufferCount)++;


				// wait untill message is fully buffered
				if (bufferCount > 3) {
					if (buffer[3] == XSENS_MESSAGE_USE_EXTENDED_LENGTH && bufferCount > 6 &&
						(bufferCount - headerCsLength) == length) {
						return XsensException::messageBuffered;
					} else if (bufferCount - headerCsLength == length) {
						return XsensException::messageBuffered;
					}
				}
				if (buffer[XSENS_MESSAGE_POSITION_PREAMBLE] != XSENS_MESSAGE_PREAMBLE ||
					buffer[XSENS_MESSAGE_POSITION_BID] != XSENS_MESSAGE_BID) {
					return XsensException::wrongMessageHeader;
				}
			}
			return XsensException::noException;
		}
	}

}


XsensException Xsens::receiveMessage(uint8_t buffer[], uint8_t* bufferCount, uint16_t bufferSize) {
	*bufferCount = 0;
	uint8_t headerCsLength = 5;
	uint16_t length = 0;
	uint8_t sum = 0;
	uint8_t strlen = 0;

	bool preambleFound = 0;

	Timer timer;
	timer.start();
	// make sure the length field gets read, and terminate when all message bytes were read
	while(1) {

		if(USE_TIMEOUT == 1 && timer.read_ms() >= XSENS_DEFAULT_TIMEOUT) {
			return XsensException::timeout;
		}
		if(_serial->readable()) {
			uint8_t c = (uint8_t) _serial->getc();
			//PRINT("%02x \r\n", c & 0xFF);
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

				if (strlen > bufferSize) {
					PRINT("\r\nXsens buffer exceeded (required: %d, actual: %d), abort.\r\n", length, bufferSize);
					// TODO voordat je returnt eerst de buffer leeg lezen
					return XsensException::messageTooLarge;
				}
				buffer[*bufferCount] = c;
				// skip preamble in checksum calculation
				if(*bufferCount > 0) {
					sum += c;
				}
				(*bufferCount)++;
				strlen++;



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



	//PRINT("\n");
	// Check if header fields have correct values
	if(	buffer[XSENS_MESSAGE_POSITION_PREAMBLE] != XSENS_MESSAGE_PREAMBLE ||
		buffer[XSENS_MESSAGE_POSITION_BID] != XSENS_MESSAGE_BID) {
		return XsensException::wrongMessageHeader;
	}

	// Check checksum
	if(sum != 0) {
		return XsensException::checksum;
	}

	return XsensException::noException;
}

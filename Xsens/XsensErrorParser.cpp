/*
 * XsensErrorParser.cpp
 *
 *  Created on: Mar 20, 2014
 *      Author: boris
 */

#include "mbed.h"
#include "../utilities/utilities.h"
#include "XsensErrorParser.h"

namespace sbt {

XsensErrorParser::XsensErrorParser(XsensException* data){
	this->data = data;
}

void XsensErrorParser::parse(uint8_t d[], int l) {
	switch(d[XSENS_ERROR_MESSAGE_ERROR_CODE]) {
		case 0x03: *data = XsensException::xsensPeriodSentInvalidRange;  break;
		case 0x04: *data = XsensException::xsensMessageSentInvalid; break;
		case 0x1E: *data = XsensException::xsensTimerOverflow; break;
		case 0x20: *data = XsensException::xsensBaudRateInvalidRange; break;
		case 0x21: *data = XsensException::xsensParameterSentInvalid; break;
	}
}

} /* namespace sbt */

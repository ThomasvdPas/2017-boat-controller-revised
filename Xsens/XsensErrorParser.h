/*
 * XsensErrorParser.h
 *
 *  Created on: Mar 20, 2014
 *      Author: boris
 */

#ifndef XSENSERRORPARSER_H_
#define XSENSERRORPARSER_H_

#include "../utilities/utilities.h"

namespace sbt {

#define XSENS_ERROR_MESSAGE_ERROR_CODE	4

class XsensErrorParser {
private:
	XsensException* data;
public:
	XsensErrorParser(XsensException* errorCode);
	void parse(uint8_t d[], int l);
};

} /* namespace sbt */

#endif /* XSENSERRORPARSER_H_ */

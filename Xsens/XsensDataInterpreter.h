/*
 * XsensDataInterperter.h
 *
 *  Created on: Mar 19, 2014
 *      Author: boris
 */

#ifndef XSENSDATAINTERPRETER_H_
#define XSENSDATAINTERPRETER_H_

#include "XsensDataParser.h"

using namespace sbt;

namespace sbt {

class XsensDataInterpreter {
private:
	xsens_data_t *d;
public:
	XsensDataInterpreter(xsens_data_t *data);

	float getAccelerationZ();
	bool getClipFlagAccZ();
	float getAccelerationX();
	float getAccelerationY();
	float getVelocity();
	float getPitch();
	float getRoll();
	float getYaw();
	float getRotX();
	float getRotY();
	float getRotZ();
	int getBaroPressure();

	void printState();

	virtual ~XsensDataInterpreter();
};

} /* namespace sbt */

#endif /* XSENSDATAINTERPRETER_H_ */

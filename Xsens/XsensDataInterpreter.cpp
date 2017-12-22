/*
 * XsensDataInterpreter.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: boris
 */

#include "Xsens.h"
#include "XsensDataInterpreter.h"

using namespace sbt;

 extern Serial pc;
extern Mutex pcMutex;

namespace sbt {

XsensDataInterpreter::XsensDataInterpreter(xsens_data_t *data) : d(data) {}

float XsensDataInterpreter::getAccelerationZ() { return d->acceleration.z; }
bool XsensDataInterpreter::getClipFlagAccZ() { return d->statusWord.clipflagAccZ; }
float XsensDataInterpreter::getAccelerationX() { return d->acceleration.x; }
float XsensDataInterpreter::getAccelerationY() { return d->acceleration.y; }
float XsensDataInterpreter::getVelocity() { return d->velocity_xy; }
float XsensDataInterpreter::getPitch() { return d->rotation.pitch; }
float XsensDataInterpreter::getRoll() { return d->rotation.roll; }
float XsensDataInterpreter::getYaw() { return d->rotation.yaw; }
float XsensDataInterpreter::getRotX() { return d->rot.x; }
float XsensDataInterpreter::getRotY() { return d->rot.y; }
float XsensDataInterpreter::getRotZ() { return d->rot.z; }
int XsensDataInterpreter::getBaroPressure() { return d->baro.pressure; }
void  XsensDataInterpreter::printState() {
	PRINT("[YPR: %2f,%2f,%2f;BARO:%d;CNT:%d]\r\n", getYaw(), getPitch(), getRoll(), getBaroPressure(), d->packetCounter.count);
}




XsensDataInterpreter::~XsensDataInterpreter() {

}

} /* namespace sbt */

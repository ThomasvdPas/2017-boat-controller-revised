/*
 * Watchdog.h
 *
 *  Created on: Mar 25, 2014
 *      Author: boris
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

class Watchdog {
public:
	void kick(float);
	void kick();
	uint8_t wasWatchDogReset();
};

#endif /* WATCHDOG_H_ */

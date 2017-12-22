/*
 * Watchdog.cpp
 *
 *  Created on: Mar 25, 2014
 *      Author: boris
 */

#include "mbed.h"
#include "Watchdog.h"

// Load timeout value in watchdog timer and enable
void Watchdog::kick(float s) {
	LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
	uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
	LPC_WDT->WDTC = s * (float)clk;
	LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
	kick();
}
// "kick" or "feed" the dog - reset the watchdog timer
// by writing this required bit pattern
void Watchdog::kick() {
	LPC_WDT->WDFEED = 0xAA;
	LPC_WDT->WDFEED = 0x55;
}

uint8_t Watchdog::wasWatchDogReset() {
	return (LPC_WDT->WDMOD >> 2) & 1;
}

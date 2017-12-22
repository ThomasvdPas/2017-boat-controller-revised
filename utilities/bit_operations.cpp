/*
 * bit_operations.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: boris
 */

#include "bit_operations.h"

bool isBitSet(uint8_t reg, int position) {
  uint8_t mask = 0x01 << position;
  return (reg & mask) != 0x00;
}

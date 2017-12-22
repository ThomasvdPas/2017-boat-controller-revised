/*
 * bit_operations.h
 *
 *  Created on: Mar 4, 2014
 *      Author: boris
 */

#ifndef BIT_OPERATIONS_H_
#define BIT_OPERATIONS_H_

#include "mbed.h"

// check if a bit at position "postition" is set
bool isBitSet(uint8_t reg, int position);

template<typename bitMaskMatchType>
// check if a bitvecor matches the mask
bool compareUsingMask(bitMaskMatchType bitArray1, bitMaskMatchType bitArray2, bitMaskMatchType mask) {
	return (bitArray1 & mask) == (bitArray2 & mask);
}

#endif /* BIT_OPERATIONS_H_ */

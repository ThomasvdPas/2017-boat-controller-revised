/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision:     V1.1.0  
*    
* Project:         CMSIS DSP Library    
* Title:        arm_abs_q15.c    
*    
* Description:    Q15 vector absolute value.    
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
*   
* Version 1.0.10 2011/7/15  
*    Big Endian support added and Merged M0 and M3/M4 Source code.   
*    
* Version 1.0.3 2010/11/29   
*    Re-organized the CMSIS folders and updated documentation.    
*     
* Version 1.0.2 2010/11/11    
*    Documentation updated.     
*    
* Version 1.0.1 2010/10/05     
*    Production release and review comments incorporated.    
*    
* Version 1.0.0 2010/09/20     
*    Production release and review comments incorporated.    
*    
* Version 0.0.7  2010/06/10     
*    Misra-C changes done    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupMath    
 */

/**    
 * @addtogroup BasicAbs    
 * @{    
 */

/**    
 * @brief Q15 vector absolute value.    
 * @param[in]       *pSrc points to the input buffer    
 * @param[out]      *pDst points to the output buffer    
 * @param[in]       blockSize number of samples in each vector    
 * @return none.    
 *    
 * <b>Scaling and Overflow Behavior:</b>    
 * \par    
 * The function uses saturating arithmetic.    
 * The Q15 value -1 (0x8000) will be saturated to the maximum allowable positive value 0x7FFF.    
 */

void arm_abs_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0

/* Run the below code for Cortex-M4 and Cortex-M3 */

  q15_t in1;                                     /* Input value1 */
  q15_t in2;                                     /* Input value2 */


  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Read two inputs */
    in1 = *pSrc++;
    in2 = *pSrc++;


    /* Store the Absolute result in the destination buffer by packing the two values, in a single cycle */

#ifndef  ARM_MATH_BIG_ENDIAN

    *__SIMD32(pDst)++ =
      __PKHBT(((in1 > 0) ? in1 : __QSUB16(0, in1)),
              ((in2 > 0) ? in2 : __QSUB16(0, in2)), 16);

#else


    *__SIMD32(pDst)++ =
      __PKHBT(((in2 > 0) ? in2 : __QSUB16(0, in2)),
              ((in1 > 0) ? in1 : __QSUB16(0, in1)), 16);

#endif /* #ifndef  ARM_MATH_BIG_ENDIAN    */

    in1 = *pSrc++;
    in2 = *pSrc++;


#ifndef  ARM_MATH_BIG_ENDIAN

    *__SIMD32(pDst)++ =
      __PKHBT(((in1 > 0) ? in1 : __QSUB16(0, in1)),
              ((in2 > 0) ? in2 : __QSUB16(0, in2)), 16);

#else


    *__SIMD32(pDst)++ =
      __PKHBT(((in2 > 0) ? in2 : __QSUB16(0, in2)),
              ((in1 > 0) ? in1 : __QSUB16(0, in1)), 16);

#endif /* #ifndef  ARM_MATH_BIG_ENDIAN    */

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Read the input */
    in1 = *pSrc++;

    /* Calculate absolute value of input and then store the result in the destination buffer. */
    *pDst++ = (in1 > 0) ? in1 : __QSUB16(0, in1);

    /* Decrement the loop counter */
    blkCnt--;
  }

#else

  /* Run the below code for Cortex-M0 */

  q15_t in;                                      /* Temporary input variable */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Read the input */
    in = *pSrc++;

    /* Calculate absolute value of input and then store the result in the destination buffer. */
    *pDst++ = (in > 0) ? in : ((in == (q15_t) 0x8000) ? 0x7fff : -in);

    /* Decrement the loop counter */
    blkCnt--;
  }

#endif /* #ifndef ARM_MATH_CM0 */

}

/**    
 * @} end of BasicAbs group    
 */

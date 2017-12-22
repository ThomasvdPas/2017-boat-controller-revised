#pragma once

#include "arm_math.h"

void kal_init();
void kal_prep();
void kal_stepone();
void kal_steptwo();
void kal_stepthree();
void kal_stepfour();
void kal_stepfive();
float kalman_filter(float xsensin, float senixin);

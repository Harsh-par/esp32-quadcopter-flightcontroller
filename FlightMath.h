#ifndef FLIGHTMATH_H
#define FLIGHTMATH_H

#include <Arduino.h>

void KalmanFilter(float &kalman_State, float &kalman_Uncertainty, float kalman_Input, float kalman_Measurement, float dt);
void LowpassFilter(float &filtered_Value, float raw_Value);

#endif
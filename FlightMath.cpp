#include "FlightMath.h"

void KalmanFilter(float &kalman_State, float &kalman_Uncertainty, float kalman_Input, float kalman_Measurement, float dt){
  kalman_State = kalman_State + dt*kalman_Input;
  kalman_Uncertainty = kalman_Uncertainty +  dt * dt * 4 * 4;
  float KalmanGain = kalman_Uncertainty * 1 / (1 * kalman_Uncertainty + 3 * 3);
  kalman_State = kalman_State+KalmanGain * (kalman_Measurement - kalman_State);
  kalman_Uncertainty = (1-KalmanGain) * kalman_Uncertainty;
}

void LowpassFilter(float &filtered_Value, float raw_Value){
  filtered_Value = filtered_Value + 0.005*(raw_Value - filtered_Value);
}
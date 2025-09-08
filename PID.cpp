#include "PID.h"
#include <Arduino.h>
#include <cmath>

PID::PID(float Kp, float Ki, float Kd, uint16_t minimum_Output, uint16_t maximum_Output) : 
  Kp(Kp), Ki(Ki), Kd(Kd), Error(0), Integral(0), Derivative(0), minimum_Output(minimum_Output), maximum_Output(maximum_Output){
    previous_Time = millis();
}

float PID::Compute(float Target, float Actual, float _Derivative, float dt){
  if(dt == 0){ dt = 0.002; }
  Error = Target - Actual;
  Derivative = _Derivative;

  Integral += Error * dt;
  Integral = constrain(Integral, -IntegralWindup, IntegralWindup); 

  float Output =  Kp * Error + Ki * Integral + Kd * Derivative;
  return constrain(Output, minimum_Output, maximum_Output);
}

float PID::Compute(float Target, float Actual, float dt){
  if(dt == 0){ dt = 0.002; }
  Error = Target - Actual;

  Derivative = (Error - previous_Error) / dt;
  previous_Error = Error;
  Integral += Error * dt;
  Integral = constrain(Integral, -IntegralWindup, IntegralWindup); 

  float Output =  Kp * Error + Ki * Integral + Kd * Derivative;
  return constrain(Output, minimum_Output, maximum_Output);
}

void PID::SetIntegralClamp(float _IntegralWindup){
  IntegralWindup = _IntegralWindup;
}

float PID::GetIntegral(void){
  return Integral;
}
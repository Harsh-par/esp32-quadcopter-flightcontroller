#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID{
  public:
    PID(float Kp, float Ki, float Kd, uint16_t minimum_Output, uint16_t maximum_Output);
    float Compute(float Target, float Actual, float _Derivative, float dt);
    float Compute(float Target, float Actual, float dt);
    void SetIntegralClamp(float _IntegralWindup);
    float GetIntegral(void);
    
  private:
    float Kp;
    float Ki;
    float Kd;
    float Error = 0;
    float previous_Error = 0;
    float Integral = 0;
    float Derivative = 0;
    float IntegralWindup = 50;
    long previous_Time = 0;

    uint16_t minimum_Output;
    uint16_t maximum_Output;
};

#endif
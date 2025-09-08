#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
  public:
    Motor(uint8_t pin_Servo, uint8_t pwm_Frequency, uint8_t pwm_Resolution, uint8_t pwm_Channel, uint16_t minimum_Throttle, uint16_t maximum_Throttle);
    void SetThrottle(uint16_t Throttle);
    uint16_t GetThrottle(void);
    void SetThrottlePercent(uint8_t Percent);
    void Arm(void);
    void Disarm(void);
    
  private:
    uint8_t  pin_Servo;
    uint8_t  pwm_Frequency;
    uint8_t  pwm_Resolution;
    uint8_t  pwm_Channel;
    float    pwm_Factor;
    uint16_t current_Throttle;
    uint16_t minimum_Throttle;
    uint16_t maximum_Throttle;

    const uint16_t disarming_Throttle = 1000;
    const uint16_t arming_Throttle = 1000;
    const int8_t minimum_Percent = 0;
    const int8_t maximum_Percent = 100;
};

#endif
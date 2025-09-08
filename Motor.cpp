#include "Motor.h"
#include <Arduino.h>
#include <cmath>

Motor::Motor(uint8_t pin_Servo, uint8_t pwm_Frequency, uint8_t pwm_Resolution, uint8_t pwm_Channel, uint16_t minimum_Throttle, uint16_t maximum_Throttle){
  this->pin_Servo         = pin_Servo;
  this->pwm_Frequency     = pwm_Frequency;
  this->pwm_Resolution    = pwm_Resolution;
  this->pwm_Channel       = pwm_Channel;
  this->pwm_Factor        = (1 << this->pwm_Resolution) * (this->pwm_Frequency) / 1000000.0;
  this->current_Throttle  = 0;
  this->minimum_Throttle  = minimum_Throttle;
  this->maximum_Throttle  = maximum_Throttle;

  //Attach motor to specific pwm Channel on esp32
  ledcAttachChannel(this->pin_Servo, this->pwm_Frequency, this->pwm_Resolution, this->pwm_Channel);
}

void Motor::SetThrottle(uint16_t Throttle){
  Throttle = constrain(Throttle, minimum_Throttle, maximum_Throttle);
  this->current_Throttle = Throttle;
  //Throttle * pwmFactor gives pulse to send to pin
  uint32_t pwm_Input = (uint32_t) (this->pwm_Factor * Throttle);
  ledcWrite(this->pin_Servo, pwm_Input);
}

uint16_t Motor::GetThrottle(void){
  return this->current_Throttle;
}

void Motor::SetThrottlePercent(uint8_t Percent){
  uint16_t Throttle = map(Percent, minimum_Percent, maximum_Percent, minimum_Throttle, maximum_Throttle);
  SetThrottle(Throttle);
}

void Motor::Arm(void){
  SetThrottle(arming_Throttle);
}

void Motor::Disarm(void){
  SetThrottle(disarming_Throttle);
}
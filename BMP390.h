#ifndef BMP390_H
#define BMP390_H

#include <Arduino.h>

class BMP390{
  public:
    BMP390(float sea_Pressure);
    void Setup();
    void Read();
    float GetAltitude();
  private:
    float Altitude;
    float sea_Pressure;
};

#endif
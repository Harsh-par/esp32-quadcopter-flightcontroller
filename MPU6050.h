#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

class MPU6050{
  public:
    MPU6050(float offset_Ax, float offset_Ay, float offset_Az, 
            float offset_Gx, float offset_Gy, float offset_Gz);
    
    void Setup();
    void Read();
    float GetAx();
    float GetAy();
    float GetAz();
    float GetGx();
    float GetGy();
    float GetGz();
  private:
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
    float offset_Ax;
    float offset_Ay;
    float offset_Az;
    float offset_Gx;
    float offset_Gy;
    float offset_Gz;

};

#endif
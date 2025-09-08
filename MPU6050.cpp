#include "MPU6050.h"
#include <Arduino.h>
#include <Wire.h>

MPU6050::MPU6050(float offset_Ax, float offset_Ay, float offset_Az, 
                 float offset_Gx, float offset_Gy, float offset_Gz) 
           : offset_Ax(offset_Ax), offset_Ay(offset_Ay), offset_Az(offset_Az), 
             offset_Gx(offset_Gx), offset_Gy(offset_Gy), offset_Gz(offset_Gz){
}

void MPU6050::Setup(){
  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x6B);             //Address of power management register
  Wire.write(0x00);             //Turning on MPU6050
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x1A);             //Address of low pass filter
  Wire.write(0x05);             //Setting to value with 10Hz Bandwidth
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x1B);             //Address of sensitivity scale factor (65.5)
  Wire.write(0x08);             //Setting to value with 10Hz Bandwidth
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x1C);             //Address accelerometer configuration
  Wire.write(0x10);             //Setting to value to 8g scale (4096)
  Wire.endTransmission();
}

void MPU6050::Read(){
  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x43);             //Address of Gyroscope X, holding upper 8 bits out of 16
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);    //Requesting 6 bytes starting from 0x43, giving all Rotational velocity values

  int16_t raw_Gx = Wire.read() << 8 | Wire.read(); //Upper 8 bits or Lower 8 bits to get 16 bit value
  int16_t raw_Gy = Wire.read() << 8 | Wire.read();
  int16_t raw_Gz = Wire.read() << 8 | Wire.read();

  Gx = (float)raw_Gx / 65.5 - offset_Gx;                       //Convert values to degrees / second
  Gy = (float)raw_Gy / 65.5 - offset_Gy;
  Gz = (float)raw_Gz / 65.5 - offset_Gz;

  Wire.beginTransmission(0x68); //Default address of MPU6050
  Wire.write(0x3B);             //Address of Accelerometer X, holding upper 8 bits out of 16
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);    //Requesting 6 bytes starting from 0x3B, giving all acceleration values
  
  int16_t raw_Ax = Wire.read() << 8 | Wire.read(); //Upper 8 bits or Lower 8 bits to get 16 bit value
  int16_t raw_Ay = Wire.read() << 8 | Wire.read();
  int16_t raw_Az = Wire.read() << 8 | Wire.read();

  Ax = (float)raw_Ax / 4096.0 - offset_Ax; //Convert values to metres / second^2 in (g) and subtracting values from manual calibration
  Ay = (float)raw_Ay / 4096.0 - offset_Ay;
  Az = (float)raw_Az / 4096.0 - offset_Az;
}

float MPU6050::GetAx(){
  return Ax;
}

float MPU6050::GetAy(){
  return Ay;
}

float MPU6050::GetAz(){
  return Az;
}

float MPU6050::GetGx(){
  return Gx;
}

float MPU6050::GetGy(){
  return Gy;
}

float MPU6050::GetGz(){
  return Gz;
}
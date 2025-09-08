#include "BMP390.h"
#include <Adafruit_BMP3XX.h>
#include <Arduino.h>
#include <Wire.h>

Adafruit_BMP3XX sensor;

BMP390::BMP390(float sea_Pressure) : sea_Pressure(sea_Pressure){
}

void BMP390::Setup(){
  if(!sensor.begin_I2C()){
  }
  sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  sensor.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  sensor.setOutputDataRate(BMP3_ODR_50_HZ);
}

void BMP390::Read(){
  if(!sensor.performReading()){
  }
  Altitude = sensor.readAltitude(sea_Pressure);
}

float BMP390::GetAltitude(){
  return Altitude;
}
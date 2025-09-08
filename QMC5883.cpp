#include <stddef.h>
#include "QMC5883.h"
#include <Arduino.h>
#include <Wire.h>

QMC5883::QMC5883(float min_Mx, float max_Mx, float min_My, float max_My, float min_Mz, float max_Mz) 
           : min_Mx(min_Mx), max_Mx(max_Mx), 
             min_My(min_My), max_My(max_My),
             min_Mz(min_Mz), max_Mz(max_Mz){
             Serial.begin(115200);
}

void QMC5883::Setup(){
  Wire.beginTransmission(0x0D); //Default address of QMC5883
	Wire.write(0x0B);             //Turning on sensor
	Wire.write(0x01);
	Wire.endTransmission();
	Wire.begin();

  //Continuous mode, 200Hz, 8g, 512
	byte Configuration = 0x01 | 0x0C | 0x10 | 0x00;
	Wire.beginTransmission(0x0D); //Default address of QMC5883
	Wire.write(0x09);             //Configuration register
	Wire.write(Configuration);
	Wire.endTransmission();
}

void QMC5883::Read(){
  Wire.beginTransmission(0x0D); //Default address of QMC5883
	Wire.write(0x00);             //Register holding mag values
	Wire.endTransmission();
	Wire.requestFrom(0x0D, 6);
	int raw_Mx = (int)(int16_t)(Wire.read() | Wire.read() << 8);
	int raw_My = (int)(int16_t)(Wire.read() | Wire.read() << 8);
	int raw_Mz = (int)(int16_t)(Wire.read() | Wire.read() << 8);

	float offset_Mx = (min_Mx + max_Mx)/2;
  float offset_My = (min_My + max_My)/2;
  float offset_Mz = (min_Mz + max_Mz)/2;

	float average_Mx = (min_Mx - max_Mx)/2;
	float average_My = (min_My - max_My)/2;
  float average_Mz = (min_Mz - max_Mz)/2;

	float average_Mag = (average_Mx + average_My + average_Mz)/3;
	float scale_Mx = average_Mag / average_Mx;
	float scale_My = average_Mag / average_My;
  float scale_Mz = average_Mag / average_Mz;

  Mx = (raw_Mx - offset_Mx)*scale_Mx;
  My = (raw_My - offset_My)*scale_My;
  Mz = (raw_Mz - offset_Mz)*scale_Mz;
}

float QMC5883::GetMx(){
  return Mx;
}

float QMC5883::GetMy(){
  return My;
}

float QMC5883::GetMz(){
  return Mz;
}

void QMC5883::ReadRaw(){
  Wire.beginTransmission(0x0D); //Default address of QMC5883
	Wire.write(0x00);             //Register holding mag values
	Wire.endTransmission();
	Wire.requestFrom(0x0D, 6);
	int raw_Mx = (int)(int16_t)(Wire.read() | Wire.read() << 8);
	int raw_My = (int)(int16_t)(Wire.read() | Wire.read() << 8);
	int raw_Mz = (int)(int16_t)(Wire.read() | Wire.read() << 8);

  Serial.print("x-> "); Serial.print(raw_Mx); Serial.print(" :y-> "); Serial.print(raw_My); Serial.print(" :z-> "); Serial.println(raw_Mz);
}
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "MPU6050.h"
#include "QMC5883.h"
#include "BMP390.h"
#include "PID.h"
#include "Motor.h"
#include "FlightMath.h"

//const byte address_MPU6050 = 0x68, address_QMC5883 = 0x0D, address_BMP390 = 0x77;
//#define flight_Mode
#define debug_Mode

#define espnow_Channel   1
#define pwm_Frequency    400
#define pwm_Resolution   16
#define pwm_ChannelA     0
#define pwm_ChannelB     1 
#define pwm_ChannelC     2 
#define pwm_ChannelD     3 
#define minimum_Throttle 1150
#define maximum_Throttle 1300
#define minimum_Output   0
#define maximum_Output   200

//Pin organization
//MotorA => TopLeft
//MotorB => TopRight
//MotorC => BottomRight
//MotorD => BottomLeft 
const byte pin_MotorA = 4, pin_MotorB = 5, pin_MotorC = 18, pin_MotorD = 19;
const byte pin_Sda = 21, pin_Scl = 22;

//Motor objects
Motor MotorA(pin_MotorA, pwm_Frequency, pwm_Resolution, pwm_ChannelA, minimum_Throttle, maximum_Throttle);
Motor MotorB(pin_MotorB, pwm_Frequency, pwm_Resolution, pwm_ChannelB, minimum_Throttle, maximum_Throttle);
Motor MotorC(pin_MotorC, pwm_Frequency, pwm_Resolution, pwm_ChannelC, minimum_Throttle, maximum_Throttle);
Motor MotorD(pin_MotorD, pwm_Frequency, pwm_Resolution, pwm_ChannelD, minimum_Throttle, maximum_Throttle);

//PID constants
const float roll_Kp  = 1, rollRate_Kp = 1, pitch_Kp = 1, pitchRate_Kp = 1, yaw_Kp = 1, alt_Kp = 1;
const float roll_Ki  = 0, rollRate_Ki = 0, pitch_Ki = 0, pitchRate_Ki = 0, yaw_Ki = 0, alt_Ki = 0;
const float roll_Kd  = 0, rollRate_Kd = 0, pitch_Kd = 0, pitchRate_Kd = 0, yaw_Kd = 0, alt_Kd = 0;

//PID objects
PID RollRate (rollRate_Kp, rollRate_Ki, rollRate_Kd, minimum_Output, maximum_Output);
PID Roll     (roll_Kp, roll_Ki, roll_Kd, minimum_Output, maximum_Output);
PID PitchRate(pitchRate_Kp, pitchRate_Ki, pitchRate_Kd, minimum_Output, maximum_Output);
PID Pitch    (pitch_Kp, pitch_Ki, pitch_Kd, minimum_Output, maximum_Output);
PID YawRate  (yaw_Kp, yaw_Ki, yaw_Kd, minimum_Output, maximum_Output);
//PID Altitude (alt_Kp, alt_Ki, alt_Kd, minimum_Output, maximum_Output);

//Sensor calibration constants
const float offset_Ax = -0.05, offset_Ay = -0.04, offset_Az = -0.04, offset_Gx = -4.90, offset_Gy = 2.12, offset_Gz = -0.52, offset_Roll = 0, offset_Pitch = 0;
const float min_Mx = -670, max_Mx = 570, min_My = -410, max_My = 1200, min_Mz = -1430, max_Mz = 1460;
const float initial_Pressure = 1007.0;

//Sensor objects
MPU6050 Accelerometer(offset_Ax, offset_Ay, offset_Az, offset_Gx, offset_Gy, offset_Gz);
QMC5883 Magnetometer (min_Mx, max_Mx, min_My, max_My, min_Mz, max_Mz);
BMP390  Barometer    (initial_Pressure);

//Measured and Filtered angles
float angle_Roll, angle_Pitch, angle_Yaw;
float kalman_Roll = 0, kalman_RollUncertainty = 2*2, kalman_Pitch = 0, kalman_PitchUncertainty = 2*2, kalman_Yaw = 0, kalman_YawUncertainty = 2*2;

//Values used for altitude holding, and filtered altitude/velocity with a low pass filter [altitude holding still being implemented]
float Altitude, VelocityZ = 0;
float filtered_Altitude = 0, filtered_VelocityZ = 0;

//Recieved values
struct ControllerData {
  int value_LJX = 0;
  int value_LJY = 0;
  int value_MJX = 0;
  int value_MJY = 0;
  int value_RJX = 0;
  int value_RJY = 0;
};

ControllerData Controller;

//Timing and Safety variables
const unsigned long failsafe_Time = 300;
unsigned long previous_Time;
unsigned long recieved_Time;
float dt;
bool Armed = false;
bool Switch = false;

void setup(){
  Serial.begin(115200);

  Wire.setClock(400000);               
  Wire.begin(pin_Sda, pin_Scl); 

  Accelerometer.Setup();
  Magnetometer.Setup();
  Barometer.Setup();

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.softAP("RX_1", "RX_1_Password", espnow_Channel, 0);
  esp_now_init();
  esp_now_register_recv_cb(ReceiveData);
  delay(100);

  previous_Time = millis();
}

void loop(){
  FlightController();
}

void FlightController(void){
  long current_Time = millis();
  dt = (current_Time - previous_Time) / 1000.0;
  previous_Time = current_Time;

  if( !Armed && (Controller.value_MJX > 30) && !Switch ){
    MotorA.Arm(); MotorB.Arm(); MotorC.Arm(); MotorD.Arm();
    Armed  = true; Switch = true;
  }

  if( (Armed && (Controller.value_MJX < -30) && Switch)  ||  (millis() - recieved_Time >= failsafe_Time) ){
    MotorA.Disarm(); MotorB.Disarm(); MotorC.Disarm(); MotorD.Disarm();
    Armed  = false; Switch = false;
  }

  #ifdef debug_Mode
    Armed = true;
  #endif

  if(Armed){
    CalculateOrientation(dt);

    float target_Roll     = Controller.value_RJX * 0.15;  
    float target_Pitch    = Controller.value_RJY * 0.15;
    float target_YawRate  = Controller.value_LJY * 0.15;  

    float target_RollRate  = Roll .Compute(target_Roll,  kalman_Roll,  dt);  
    float target_PitchRate = Pitch.Compute(target_Pitch, kalman_Pitch, dt); 

    float roll_Output     = RollRate .Compute(target_RollRate,  Accelerometer.GetGx(), dt);
    float pitch_Output    = PitchRate.Compute(target_PitchRate, Accelerometer.GetGy(), dt);
    float yaw_Output      = YawRate  .Compute(target_YawRate,   Accelerometer.GetGz(), dt);
    float throttle_Output = (Controller.value_LJX > 0) ? minimum_Throttle + (maximum_Throttle - minimum_Throttle)*(Controller.value_LJX / 100.0f) : minimum_Throttle;

    float throttle_A = throttle_Output - roll_Output + pitch_Output + yaw_Output;
    float throttle_B = throttle_Output + roll_Output + pitch_Output - yaw_Output;
    float throttle_C = throttle_Output + roll_Output - pitch_Output + yaw_Output;
    float throttle_D = throttle_Output - roll_Output - pitch_Output - yaw_Output;

    #ifdef flight_Mode
    MotorA.SetThrottle(throttle_A);
    MotorB.SetThrottle(throttle_B);
    MotorC.SetThrottle(throttle_C);
    MotorD.SetThrottle(throttle_D);
    #endif
    
    #ifdef debug_Mode
      static int debug_LoopCounter = 0;
      debug_LoopCounter++;
      if(debug_LoopCounter >= 50) { 
        //Serial.print("dt : "); Serial.print(dt, 4); Serial.println();
        SerialData("A", throttle_A);
        SerialData("B", throttle_B);
        SerialData("C", throttle_C);
        SerialData("D", throttle_D);
        debug_LoopCounter = 0;
      }
    #endif
  }
}

void CalculateOrientation(float dt){
  Accelerometer.Read();
  //Magnetometer.Read();
  //Barometer.Read();

  float Ax = Accelerometer.GetAx();
  float Ay = Accelerometer.GetAy();
  float Az = Accelerometer.GetAz();
  float Gx = Accelerometer.GetGx();
  float Gy = Accelerometer.GetGy();
  float Gz = Accelerometer.GetGz();
  //float Mx = Magnetometer.GetMx();
  //float My = Magnetometer.GetMy();
  //float Mz = Magnetometer.GetMz();
  //Altitude = Barometer.GetAltitude();

  angle_Roll  = atan(  Ay / sqrt(Ax*Ax + Az*Az) ) * (180.0 / PI);
  angle_Pitch = atan( -Ax / sqrt(Ay*Ay + Az*Az) ) * (180.0 / PI);
  //angle_Yaw   = atan2(My, Mx) * (180.0 / PI);
  //angle_Yaw   = (int)angle_Yaw % 360;

  KalmanFilter(kalman_Roll, kalman_RollUncertainty, Gx, angle_Roll, dt);
  KalmanFilter(kalman_Pitch, kalman_PitchUncertainty, Gy, angle_Pitch, dt);
  //KalmanFilter(kalman_Yaw, kalman_YawUncertainty, Gz, angle_Yaw, dt);

  /*float Az_Inertial = (Az*cos(kalman_Pitch*DEG_TO_RAD)*cos(kalman_Roll*DEG_TO_RAD) - 1)*9.81;
  VelocityZ = VelocityZ + Az_Inertial*dt;

  LowpassFilter(filtered_Altitude, Altitude);
  LowpassFilter(filtered_Altitude, Altitude);
  LowpassFilter(filtered_VelocityZ, VelocityZ);
  LowpassFilter(filtered_VelocityZ, VelocityZ);
  */
}

void SerialData(String Label, float Data){
  Serial.print(Label); Serial.print(" : "); Serial.println(Data);
}

void ReceiveData(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int data_len){
  memcpy(&Controller, data, sizeof(Controller));
  recieved_Time = millis();
}
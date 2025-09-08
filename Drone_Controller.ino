#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>

#define EspnowChannel 1
#define ScreenAddress 0x3C 

esp_now_peer_info_t Reciever;

const int pin_Sda = 21;
const int pin_Scl = 22;
const int pin_RJX = 32;
const int pin_RJY = 33;
const int pin_MJX = 34;
const int pin_MJY = 35;
const int pin_LJX = 36;
const int pin_LJY = 39;

int value_LJX = 0;
int value_LJY = 0;
int value_MJX = 0;
int value_MJY = 0;
int value_RJX = 0;
int value_RJY = 0;

const String wifi_Network    = "RX_1";
const String wifi_Password   = "RX_1_Password";

struct ControllerData {
  int value_LJX = 0;
  int value_LJY = 0;
  int value_MJX = 0;
  int value_MJY = 0;
  int value_RJX = 0;
  int value_RJY = 0;
};

ControllerData Controller;

Adafruit_SSD1306 LedDisplay(128, 32, &Wire, -1); //ScreenWidth, Height, ..., reset
 
void setup(){
  Serial.begin(115200);

  WiFi.begin(wifi_Network, wifi_Password);
  Wire.begin(pin_Sda, pin_Scl);

  pinMode(pin_LJX, INPUT);
  pinMode(pin_LJY, INPUT);
  pinMode(pin_MJX, INPUT);
  pinMode(pin_MJY, INPUT);
  pinMode(pin_RJX, INPUT);
  pinMode(pin_RJY, INPUT);

  if(!LedDisplay.begin(SSD1306_SWITCHCAPVCC, ScreenAddress)) {
  Serial.println(F("SSD1306 allocation failed")); for(;;); 
  } 
  while(WiFi.status() != WL_CONNECTED){
  WriteText("Connecting to Wifi", " ", " ");
  } 

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  ScanForSlave();
  esp_now_add_peer(&Reciever);
  WriteText("ESPNOW Connected to Peer", " ", " ");
  delay(1000);
}

void loop(){
  ReadJoystick();
  SendJoystickData();
}

//Functions for Joystick reading handling
int ProcessJoystick(int Raw, int Center = 1800,  int RangeNeg = 1800, int RangePos = 2000, int Deadzone = 100){
  int Value = Raw - Center;
  if(abs(Value) < Deadzone) return 0;
  if(Value < 0)
    Value = map(Value, -RangeNeg, 0, -100, 0);
  else
    Value = map(Value, 0, RangePos, 0, 100);
  return constrain(Value, -100, 100);
}

void ReadJoystick(void){
  value_LJX = ProcessJoystick(analogRead(pin_LJX));
  value_LJY = ProcessJoystick(analogRead(pin_LJY));
  value_MJX = ProcessJoystick(analogRead(pin_MJX));
  value_MJY = ProcessJoystick(analogRead(pin_MJY));
  value_RJX = ProcessJoystick(analogRead(pin_RJX));
  value_RJY = ProcessJoystick(analogRead(pin_RJY));

  String L = String(value_LJX) + " : " + String(value_LJY);
  String M = String(value_MJX) + " : " + String(value_MJY);
  String R = String(value_RJX) + " : " + String(value_RJY);
  WriteText(L, M, R);
}

void SendJoystickData(void){
  Controller.value_LJX = value_LJX;
  Controller.value_LJY = value_LJY;
  Controller.value_MJX = value_MJX;
  Controller.value_MJY = value_MJY;
  Controller.value_RJX = value_RJX;
  Controller.value_RJY = value_RJY;
  esp_now_send(Reciever.peer_addr, (uint8_t *)&Controller, sizeof(Controller));
}

//Functions for OLED Display
void WriteText(String line1, String line2, String line3){
  LedDisplay.clearDisplay();
  LedDisplay.setTextSize(1);
  LedDisplay.setTextColor(SSD1306_WHITE);
  LedDisplay.setCursor(0, 0);
  LedDisplay.println(line1);
  LedDisplay.setCursor(0, 10);
  LedDisplay.println(line2);
  LedDisplay.setCursor(0, 20);
  LedDisplay.println(line3);
  LedDisplay.display();
}

//Functions for ESPNOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("Data sent");
}

void ScanForSlave(void){
  int8_t scanResults = WiFi.scanNetworks();
  for (int i = 0; i < scanResults; ++i) {
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);
    if (SSID.indexOf("RX") == 0) {
      int mac[6];
      if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
      for (int ii = 0; ii < 6; ++ii) { Reciever.peer_addr[ii] = (uint8_t)mac[ii]; }
      }
      Reciever.channel = EspnowChannel; // pick a channel
      Reciever.encrypt = 0; // no encryption
      break;
    }
  }
}
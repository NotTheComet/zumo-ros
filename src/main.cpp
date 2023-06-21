#include <Arduino.h>
#include "ArduinoJson.h"

#include <Wire.h>
#include <Zumo32U4.h>
#include <PID_v1.h>

//Constants 
const double r = 0.00198; // radius of the wheel
const double cpr = 909.7;// counts per revoulution
const double l = 9.90; // length from wheel to wheel
const float rpm_to_radians = 0.10471975512; // rpm to radians
const float rad_to_deg = 57.29578; // radians to degrees

//Encoder Values
int16_t EncoderL;
int16_t EncoderR;

//Zumo Control
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU  IMU;
Zumo32U4OLED oled;

// X and Y coordinates and rotation(thea)
int x;
int y;
double theta;

const int32_t turnAngle1 = (0x20000000 + 22) / 45;

float rpmR;
float rpmL;

float velR;
float velL;

double oldcountsR = 0;
double oldcountsL = 0;

long previousMillis = 0;
long currentMillis = 0;

DynamicJsonDocument  data(1024);


void odom() {

  EncoderL = encoders.getCountsLeft();
  EncoderR = encoders.getCountsRight();
  IMU.read();

  if (EncoderL or EncoderR > 0) {
    double dl = 2 * PI * r * (EncoderL/cpr);
    double dr = 2 * PI * r * (EncoderR/cpr);

    double dc = (dl + dr) / 2;

    //theta = (180 / PI) * (dl - dr) / l;  
    theta = (IMU.g.z / turnAngle1) * (180/ PI);

    x = dc * (180/PI) * sin(theta);
    y = dc * (180/PI) * cos(theta);
  }

  data["x"] = x;
  data["y"] = y;
  data["theta"] = theta;
}

void displayINFO() {
  oled.clear();
  oled.gotoXY(0,0);
  //oled.print('X: ');
  oled.print(x);

  oled.gotoXY(0,1);
  //oled.print('Y: ');
  oled.print(y);
}

void move() {

}

void setup() {
  Serial.begin(9600);
  //Serial1.begin(115200);
  
  Wire.begin();
  IMU.init();
  IMU.enableDefault();
  oled.init();
}

void loop() {
  currentMillis = millis();
  odom();
  displayINFO();

  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;

    double countsR =  EncoderR;
    double countsL = EncoderL;
    
    rpmL = (float)(countsL * 60 / cpr);
    rpmR = (float)(countsR * 60 / cpr);

    if (oldcountsL != countsL) {
      velL = rpmL * rpm_to_radians;
    } else {
      velL = 0;
    }

    if (oldcountsR != countsR) {
      velR = rpmR * rpm_to_radians;
    } else {
      velR = 0;
    }

    oldcountsR = countsR;
    oldcountsL = countsL;
  
    data["velR"] = velR;
    data["velL"] = velL;
  }


  serializeJson(data, Serial);
  Serial.println("\n");
}


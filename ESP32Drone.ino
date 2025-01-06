#include "MPU6050.h"
#include "wificomms.h"
#include <ESP32Servo.h>
#include "pid.h"

wificomms net;
pid pidcalc;

int m1_drive, m2_drive, m3_drive, m4_drive;

// mpu6050 pins are SCL (D22) & SDA (D21)

// motor pins
const int mot1_pin = 13; // right back
const int mot2_pin = 12; // right front
const int mot3_pin = 14; // left front
const int mot4_pin = 27; // left back

void setup() {
  net.connect();
  net.listen();

  pidcalc.initialise();
  
  ledcAttach(mot1_pin, 12000, 8);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(mot2_pin, 12000, 8);
  ledcAttach(mot3_pin, 12000, 8);
  ledcAttach(mot4_pin, 12000, 8);
}

void loop() {
  int pitch, roll, yaw, throttle;
  
  net.read(pitch, roll, yaw, throttle);

  pidcalc.calculate(pitch, roll, yaw, throttle);
  
  float pitch_PID, roll_PID, yaw_PID;
  pidcalc.getValues(pitch_PID, roll_PID, yaw_PID);

  m1_drive = input_THROTTLE + pitch_PID - roll_PID + yaw_PID;
  m2_drive = input_THROTTLE + pitch_PID + roll_PID - yaw_PID;
  m3_drive = input_THROTTLE - pitch_PID + roll_PID + yaw_PID;
  m4_drive = input_THROTTLE - pitch_PID - roll_PID - yaw_PID;

  // apply PWM
  ledcWrite(mot1_pin, m1_drive); 
  ledcAttach(mot2_pin, m2_drive);
  ledcAttach(mot3_pin, m3_drive);
  ledcAttach(mot4_pin, m4_drive);

}

#include "MPU6050.h"
#include "Wire.h"

void mpu6050::initialise()
{
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  //accel
  Wire.write(0x08);  //+-4g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  //gyro
  Wire.write(0x18);  //2000dps
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void mpu6050::calibrate()
{
  for (int cal_int = 0; cal_int < 4000; cal_int++) {
    if (cal_int % 125 == 0) Serial.print(".");
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14)
      ;
    acc_y = Wire.read() << 8 | Wire.read();
    acc_x = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delayMicroseconds(100);
  }
  gyro_x_cal /= 4000;
  gyro_y_cal /= 4000;
  gyro_z_cal /= 4000;
}

void mpu6050::read()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14)
    ;
  acc_y = Wire.read() << 8 | Wire.read();
  acc_x = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}


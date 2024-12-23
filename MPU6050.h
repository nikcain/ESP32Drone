#ifndef mpu6050_h
#define mpu6050_h

#include "Arduino.h"

class mpu6050 {
  public:
    void initialise();
    void calibrate();

    void read();
    void getAcceleration(int16_t& x, int16_t& y, int16_t& z) {x=acc_x;y=acc_y;z=acc_z;}
    void getRotation(int16_t& x, int16_t& y, int16_t& z) {x=gyro_x;y=gyro_y;z=gyro_z;}

  private:
  
    int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature;
    long gyro_x_cal;
    long gyro_y_cal;
    long gyro_z_cal;
};

#endif // mpu6050_h
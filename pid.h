#ifndef pid_h
#define pid_h

#include "MPU6050.h"

class pid
{
  public:
    void initialise();
    void calculate(int pitch_, int roll_, int yaw_, int throttle_);
    void getValues(float& pitch_PID_, float& roll_PID_, float& yaw_PID_)
    { pitch_PID_ = pitch_PID; roll_PID_ = roll_PID; yaw_PID_ = yaw_PID; }

  private:
    mpu6050 gyro;
    long Time, timePrev;
    float elapsedTime;
    float pitch_PID, roll_PID, yaw_PID;

    float angle_pitch_output, angle_roll_output, angle_yaw_output;

    float roll_error, roll_previous_error;
    float pitch_error, pitch_previous_error;
    float yaw_error, yaw_previous_error;

    float roll_pid_p, roll_pid_d, roll_pid_i;
    float pitch_pid_p, pitch_pid_i, pitch_pid_d;
    float yaw_pid_p, yaw_pid_i, yaw_pid_d;

    double twoX_kp = 5;      //5
    double twoX_ki = 0.003;  //0.003
    double twoX_kd = 1.4;    //1.4
    double yaw_kp = 8;       //5
    double yaw_ki = 0;       //0.005
    double yaw_kd = 4;       //2.8
};


#endif // pid_h
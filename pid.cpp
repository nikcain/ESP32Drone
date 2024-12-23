#include "pid.h"

void pid::initialise()
{
  gyro.initialise();
  gyro.calibrate(); 
}

void pid::calculate(int pitch_, int roll_, int yaw_, int throttle_)
{
  int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
  float angle_pitch, angle_roll, angle_yaw, prev_roll, prev_pitch, prev_yaw;
  float angle_roll_acc, angle_pitch_acc;

  gyro.read();
  gyro.getAcceleration(acc_x, acc_y, acc_z);
  gyro.getRotation(gyro_x, gyro_y, gyro_z);

  timePrev = Time;
  Time = micros();
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  

  float acceleration_x, acceleration_y, acceleration_z;
  acceleration_x = gyro_x * (-0.0610687023);
  acceleration_y = gyro_y * (-0.0610687023);
  acceleration_z = gyro_z * (-0.0610687023);
  angle_pitch += acceleration_x * elapsedTime;
  angle_roll += acceleration_y * elapsedTime;
  angle_yaw += acceleration_z * elapsedTime;
  if (angle_yaw >= 180.00) {
    angle_yaw -= 360;
  } else if (angle_yaw < -180.00) {
    angle_yaw += 360;
  }
  angle_roll_acc = atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * (-57.296);
  angle_pitch_acc = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.296;
  angle_pitch_acc -= 4;
  angle_roll_acc += 9;
  //????
  boolean set_gyro_angles = true;
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;
    
  float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i, yaw_pid_d;

  float roll_desired_angle = 3 * (roll_ - 50) / 10.0;
  float pitch_desired_angle = 3 * (pitch_ - 50) / 10.0;
  
  float P_factor = 0.001286376 * throttle_ + 0.616932;

  roll_error = angle_roll_output - roll_desired_angle;
  pitch_error = angle_pitch_output - pitch_desired_angle;
  yaw_error = angle_yaw_output;

  roll_pid_p = P_factor * twoX_kp * roll_error;
  pitch_pid_p = P_factor * twoX_kp * pitch_error;
  yaw_pid_p = yaw_kp * yaw_error;

  roll_pid_i += twoX_ki * roll_error;
  pitch_pid_i += twoX_ki * pitch_error;
  yaw_pid_i += yaw_ki * yaw_error;

  roll_pid_d = twoX_kd * acceleration_y;
  pitch_pid_d = twoX_kd * acceleration_x;
  yaw_pid_d = yaw_kd * acceleration_z;

  if (roll_pid_i > 0 && roll_error < 0) {
    roll_pid_i = 0;
  } else if (roll_pid_i < 0 && roll_error > 0) {
    roll_pid_i = 0;
  }
  if (pitch_pid_i > 0 && pitch_error < 0) {
    pitch_pid_i = 0;
  } else if (pitch_pid_i < 0 && pitch_error > 0) {
    pitch_pid_i = 0;
  }
  if (yaw_pid_i > 0 && yaw_error < 0) {
    yaw_pid_i = 0;
  } else if (yaw_pid_i < 0 && yaw_error > 0) {
    yaw_pid_i = 0;
  }

  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;
}

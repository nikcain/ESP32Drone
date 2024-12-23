#include "pid.h"

void pid::calculate(int pitch_, int roll_, int yaw_, int throttle_)
{
  roll_desired_angle = 3 * (input_ROLL - 50) / 10.0;
  pitch_desired_angle = 3 * (input_PITCH - 50) / 10.0;
  
  P_factor = 0.001286376 * input_THROTTLE + 0.616932;

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

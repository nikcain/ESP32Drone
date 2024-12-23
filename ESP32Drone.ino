#include "MPU6050.h"
#include "wificomms.h"
#include "pid.h"

wificomms net;
pid pidcalc;
hw_timer_t * PWMTimer = NULL;

int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW = 50;
volatile int input_THROTTLE = 0;

// PWM cycle
uint8_t pwm_stops;
int pulldown_time_temp[] = { 0, 0, 0, 0, 0 };
int pulldown_time[] = { 0, 0, 0, 0, 0 };
volatile int pulldown_time_temp_loop[] = { 0, 0, 0, 0, 0 };  //volatile key

// for controlling motors via registry (!)
volatile int order[] = { 0, 0, 0, 0 };  //volatile key
uint8_t pin[] = { 14, 12, 13, 15 };
int ESCout_1, ESCout_2, ESCout_3, ESCout_4;

void setup() {
  net.connect();
  net.listen();
  pidcalc.initialise();
  
  //can be set to 80 MHz, 160 MHz, or 240 MHz
  PWMTimer = timerBegin(80000); // timer frequency
  timerAttachInterrupt(PWMTimer, PWMcall);
  //       TIM_DIV1      80MHz => 80 ticks/µs => Max: (2^23 / 80) µs =  ~0.105s
  //       TIM_LOOP    1 => regular interrupt 
  //timerEnable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
}

void PWMcall(){
  switch (pwm_stops) {
    case 0:
      pulldown_time_temp[0] = pulldown_time_temp_loop[0];
      pulldown_time_temp[1] = pulldown_time_temp_loop[1];
      pulldown_time_temp[2] = pulldown_time_temp_loop[2];
      pulldown_time_temp[3] = pulldown_time_temp_loop[3];
      pulldown_time_temp[4] = pulldown_time_temp_loop[4];
      pwm_stops = 1;
   /*   if (input_THROTTLE != 0) {
        GPOS = (1 << 14);
        GPOS = (1 << 12);
        GPOS = (1 << 15);
        GPOS = (1 << 13);
      }
      */
      timerWrite(PWMTimer, 80 * pulldown_time_temp[0]);
      break;
    case 1:
      pwm_stops = 2;
   //   GPOC = (1 << pin[order[0]]);
      timerWrite(PWMTimer, 80 * pulldown_time_temp[1]);
      break;
    case 2:
      pwm_stops = 3;
    //  GPOC = (1 << pin[order[1]]);
      timerWrite(PWMTimer, 80 * pulldown_time_temp[2]);
      break;
    case 3:
      pwm_stops = 4;
   //   GPOC = (1 << pin[order[2]]);
      timerWrite(PWMTimer, 80 * pulldown_time_temp[3]);
      break;
    case 4:
      pwm_stops = 0;
    //  GPOC = (1 << pin[order[3]]);
      timerWrite(PWMTimer, 80 * pulldown_time_temp[4]);
      break;
  }
}

void cycleMotors()
{
  int arr[] = { 20, 10, 20, 10 };
  volatile int order[] = { 0, 0, 0, 0 };  //volatile key
  int temp_arr[] = { 0, 0, 0, 0 };
  boolean orderState1, orderState2, orderState3, orderState4, Timer_Init;

  if (ESCout_1 > 1199) ESCout_1 = 1199;
  else if (ESCout_1 < 1) ESCout_1 = 1;
  if (ESCout_2 > 1199) ESCout_2 = 1199;
  else if (ESCout_2 < 1) ESCout_2 = 1;
  if (ESCout_3 > 1199) ESCout_3 = 1199;
  else if (ESCout_3 < 1) ESCout_3 = 1;
  if (ESCout_4 > 1199) ESCout_4 = 1199;
  else if (ESCout_4 < 1) ESCout_4 = 1;

  //----------------------------- Sorting -------------------------------//
  arr[0] = ESCout_1;
  arr[1] = ESCout_2;
  arr[2] = ESCout_3;
  arr[3] = ESCout_4;
  temp_arr[0] = arr[0];
  temp_arr[1] = arr[1];
  temp_arr[2] = arr[2];
  temp_arr[3] = arr[3];
  int temp, temp_i;
  for (int i = 0; i < 3; i++) {
    temp_i = i;
    for (int j = i + 1; j < 4; j++)
      if (temp_arr[j] < temp_arr[temp_i])
        temp_i = j;
    temp = temp_arr[temp_i];
    temp_arr[temp_i] = temp_arr[i];
    temp_arr[i] = temp;
  }
  pulldown_time[0] = temp_arr[0];
  pulldown_time[1] = temp_arr[1] - temp_arr[0];
  pulldown_time[2] = temp_arr[2] - temp_arr[1];
  pulldown_time[3] = temp_arr[3] - temp_arr[2];
  pulldown_time[4] = 1200 - temp_arr[3];
  if (pulldown_time[1] == 0) { pulldown_time[1] = 1; }
  if (pulldown_time[2] == 0) { pulldown_time[2] = 1; }
  if (pulldown_time[3] == 0) { pulldown_time[3] = 1; }
  if (pulldown_time[4] == 0) { pulldown_time[4] = 1; }
  pulldown_time_temp_loop[0] = pulldown_time[0];
  pulldown_time_temp_loop[1] = pulldown_time[1];
  pulldown_time_temp_loop[2] = pulldown_time[2];
  pulldown_time_temp_loop[3] = pulldown_time[3];
  pulldown_time_temp_loop[4] = pulldown_time[4];
  orderState1 = false;
  orderState2 = false;
  orderState3 = false;
  orderState4 = false;
  for (int k = 0; k < 4; k++) {
    if (temp_arr[0] == arr[k] && orderState1 == false) {
      order[0] = k;
      orderState1 = true;
    } else if (temp_arr[1] == arr[k] && orderState2 == false) {
      order[1] = k;
      orderState2 = true;
    } else if (temp_arr[2] == arr[k] && orderState3 == false) {
      order[2] = k;
      orderState3 = true;
    } else if (temp_arr[3] == arr[k] && orderState4 == false) {
      order[3] = k;
      orderState4 = true;
    }
  }
}

void loop() {
  int roll, yaw, pitch, throttle;
  
  net.read(roll, pitch, throttle);

  pidcalc.calculate(pitch, roll, yaw, throttle);
  
  float pitch_PID, roll_PID, yaw_PID;
  pidcalc.getValues(pitch_PID, roll_PID, yaw_PID);
  ESCout_1 = input_THROTTLE + pitch_PID - roll_PID + yaw_PID;
  ESCout_2 = input_THROTTLE + pitch_PID + roll_PID - yaw_PID;
  ESCout_3 = input_THROTTLE - pitch_PID + roll_PID + yaw_PID;
  ESCout_4 = input_THROTTLE - pitch_PID - roll_PID - yaw_PID;

  cycleMotors();
}

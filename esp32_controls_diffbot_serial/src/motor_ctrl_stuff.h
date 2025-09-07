

#ifndef MOTORS_STUFF_
#define MOTORS_STUFF_
#include <Arduino.h>
#include "encoders_stuff.h"

#define LEFT 0
#define RIGHT 1

// max speed is 85 cms.

// pins

#define RIGHT_MOTOR_PWM 16 // front right motor
#define RIGHT_MOTOR_DIR 5  //
#define LEFT_MOTOR_PWM 17  // front left motor
#define LEFT_MOTOR_DIR 19  //

// back motors (Back of board)

#define RIGHT_MOTOR_PWM2 13 //
#define RIGHT_MOTOR_DIR2 12 // back right motor
#define LEFT_MOTOR_PWM2 26  // back left motor
#define LEFT_MOTOR_DIR2 14  //

#define LEFT_FWD HIGH
#define RIGHT_FWD LOW
#define LEFT_BACK LOW
#define RIGHT_BACK HIGH

QueueHandle_t dist_queue = NULL;
QueueHandle_t dist_queue_int;
QueueHandle_t theta_queue;
QueueHandle_t theta_queue_int; // inetrrupt
QueueHandle_t speed_queue = NULL;

TaskHandle_t set_robot_target_speed_task;
TaskHandle_t serial_command_process_task;

TaskHandle_t move_dist_task;
TaskHandle_t move_theta_task;

TaskHandle_t robot_move;

QueueHandle_t queue;
QueueHandle_t queue_ret;
SemaphoreHandle_t Semaphore_prev_time;
UBaseType_t n_messages;

SemaphoreHandle_t Semaphore_pwm;

// bool emergency_stop = false;

uint8_t pwm_pins[] = {LEFT_MOTOR_PWM, LEFT_MOTOR_PWM2, RIGHT_MOTOR_PWM, RIGHT_MOTOR_PWM2};
uint8_t dir_pins[] = {LEFT_MOTOR_DIR, LEFT_MOTOR_DIR2, RIGHT_MOTOR_DIR, RIGHT_MOTOR_DIR2};

// pwm paramters
const int mtr_left_pwm_channel = 8;
const int mtr_right_pwm_channel = 4;
const int mtr_left_pwm2_channel = 5;
const int mtr_right_pwm2_channel = 6;
const int lresolution = 8;
const int freq = 4000;

unsigned long max_motor_speed = 82; // max speed of motor in cms

const int pwm_channels[] = {mtr_left_pwm_channel, mtr_left_pwm2_channel, mtr_right_pwm_channel, mtr_right_pwm2_channel};


volatile int motor_spds[] = {20, 20, 20, 20};
volatile int motor_spds_pwm[] = {100, 100, 100, 100};

bool emergency_stop = false; // if true, robot will stop and not move until cleared.

float Kp = 2.0;
float Kd = 0.5;
float Ki = 0.1;

// Placeholder for functions

void robot_setup();
void robot_stop();


int set_target_speed(float target, int motor, int init_spd);


int mtr_lft_state[] = {LEFT_FWD, LEFT_BACK, LEFT_BACK, LEFT_FWD, LEFT_FWD, LEFT_FWD, LEFT_BACK, LEFT_BACK}; // fwd, rev, lft, right
int mtr_rgt_state[] = {RIGHT_FWD, RIGHT_BACK, RIGHT_FWD, RIGHT_BACK, RIGHT_FWD, RIGHT_FWD, RIGHT_BACK, RIGHT_BACK};

// void checkSerialCommands() {
//   if (Serial.available()) {
//     String cmd = Serial.readStringUntil('\n');
//     String response = "No response";
//     Serial.println(response);
//   }
// }

void robot_stop()
{
  // stop all motors
  emergency_stop = true;
  xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);

  for (int i = 0; i < 4; i++)
  {
    ledcWrite(pwm_channels[i], 0);
    motor_spds_pwm[i] = 0;
    motor_spds[i] = 0;
  }

  xSemaphoreGive(Semaphore_pwm);
}

void robot_setup()
{
  // Pins for Motor Controller
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM2, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM2, OUTPUT);

  // Motor uses PWM Channel 8
  ledcSetup(mtr_left_pwm_channel, freq, lresolution);
  ledcAttachPin(LEFT_MOTOR_PWM, mtr_left_pwm_channel);
  ledcWrite(mtr_left_pwm_channel, 0);

  ledcSetup(mtr_right_pwm_channel, freq, lresolution);
  ledcAttachPin(RIGHT_MOTOR_PWM, mtr_right_pwm_channel);
  ledcWrite(mtr_right_pwm_channel, 0);

  ledcSetup(mtr_left_pwm2_channel, freq, lresolution);
  ledcAttachPin(LEFT_MOTOR_PWM2, mtr_left_pwm2_channel);
  ledcWrite(mtr_left_pwm2_channel, 0);

  ledcSetup(mtr_right_pwm2_channel, freq, lresolution);
  ledcAttachPin(RIGHT_MOTOR_PWM2, mtr_right_pwm2_channel);
  ledcWrite(mtr_right_pwm2_channel, 0);

  speed_queue = xQueueCreate(1, sizeof(int)*4);

  Semaphore_pwm = xSemaphoreCreateMutex();
}

void setup_target_speed(int *its, int *motor_s, float *target)
{
  *its = 0; // reset iterations
  reset_encoders();
  // memcpy(target, motor_s, sizeof(motor_s));

  for (int i = 0; i < 4; i++)
  {

    digitalWrite(dir_pins[i], (motor_s[i] >= 0) ? HIGH : LOW);
    target[i] = abs((float)motor_s[i]); // set target speed
  }
  if (emergency_stop)
  {
    emergency_stop = false;
    xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);
    for (int i = 0; i < 4; i++)
    {
      motor_spds_pwm[i] = (int)(((float)target[i]) * 255.0 / 82.0); // reset motor speeds
    }
    xSemaphoreGive(Semaphore_pwm);

    Serial.println("Cleared emergency stop, resuming");
  }
}

void set_robot_target_speed(void *params)
{

  int motor_s[] = {10, 10, 10, 10};
  float dist = 0.0; // myparams->dist;
  bool stop;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  // pwm_pins
  // dir_pins
  unsigned long timer_strt;
  unsigned long timer_curr;

  int its = 0;

  // uint8_t pwm_pin = pwm_pins[motor];
  // uint8_t dir_pin = dir_pins[motor];

  // const int pwm_channel = pwm_channels[motor];

  // run for 200ms, do encoder count

  float tol = 0.1;
  float error[4] = {0.0, 0.0, 0.0, 0.0};

  float speed[] = {0.0, 0.0, 0.0, 0.0};

  long cnts_prev[4];
  long cnts[4];

  float iterm = 0;
  float prev_error[4] = {0.0, 0.0, 0.0, 0.0}; // previous error for each motor
  float prev_speed[4];
  float pulse_dist = circum / ppr; // distance per pulse
  float target[4];                 // target speed in cm/s

  while (true)
  {
    // Serial.println("in move dist");
    xQueueReceive(speed_queue, &motor_s, portMAX_DELAY);

    setup_target_speed(&its, motor_s, target);
    // delay(1);
    // Serial.println("Speeds received: ");
    // Serial.print(motor_s[0]); Serial.print(", ");
    // Serial.print(motor_s[1]); Serial.print(", ");  
    // Serial.print(motor_s[2]); Serial.print(", ");
    // Serial.println(motor_s[3]);
    // its = 0; // reset iterations
    // reset_encoders();
    // // memcpy(target, motor_s, sizeof(motor_s));

    // for (int i = 0; i < 4; i++)
    // {

    //   digitalWrite(dir_pins[i], (motor_s[i] >= 0) ? HIGH : LOW);
    //   target[i] = abs((float)motor_s[i]); // set target speed
    // }
    // if (emergency_stop)
    // {
    //   emergency_stop = false;
    //   xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);
    //   for (int i = 0; i < 4; i++)
    //   {
    //     motor_spds_pwm[i] = (int)(((float)target[i]) * 255.0 / 80.0); // reset motor speeds
    //   }
    //   xSemaphoreGive(Semaphore_pwm);
      
    //   Serial.println("Cleared emergency stop, resuming");
    // }

    while (!emergency_stop && abs(speed[0] - target[0]) > tol &&
           abs(speed[1] - target[1]) > tol && abs(speed[2] - target[2]) > tol && abs(speed[3] - target[3]) > tol)
    {

      for (int i = 0; i < 4; i++)
      {
        cnts_prev[i] = get_encoder_pos(i);
        xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);

        // need to inclulde a semaphore here to stop motors

        ledcWrite(pwm_channels[i], motor_spds_pwm[i]);

        // its = 20;                      // set iterations to 20 to stop motors
        // ledcWrite(pwm_channels[i], 0); // stop motors if not moving

        xSemaphoreGive(Semaphore_pwm);
      }
      
      timer_strt = millis();
      // delay(5000); // wait for 100ms
      // wait for 30ms
      vTaskDelay(30 / portTICK_PERIOD_MS);

      for (int i = 0; i < 4; i++)
      {

        cnts[i] = get_encoder_pos(i);

        // Serial.print("Cnts ");
        //   Serial.print(i);  
        //   Serial.print(": ");
        //   Serial.println(cnts[i]);

        speed[i] = ((float)abs(cnts[i] - cnts_prev[i])) * pulse_dist * 1000 / ((float)millis() - timer_strt);

        // Serial.print("Speed ");
        // Serial.print(i);  
        // Serial.print(": ");
        // Serial.println((int)speed[i]);

        // Serial.print(", Target: ");
        // Serial.println((int)target[i]);

        error[i] = target[i] - speed[i];

        if (its == 0)
        {
          prev_speed[i] = speed[i]; // initialise previous speed
          prev_error[i] = 0;// error[i];
        }

        // iterm += Ki*error;
        iterm = Ki * prev_error[i]; // using momentum for the i-term instead.

        // Serial.print(", error: ");
        // Serial.println((int)((error[i]) * Kp + Kd * (speed[i] - prev_speed[i]) + iterm));
        motor_spds_pwm[i] += (int)(error[i] * Kp + Kd * (speed[i] - prev_speed[i]) + iterm);
        // init_spd -= (int) (Kd*(speed-prev_speed));

        if (motor_spds_pwm[i] > 255)
        {
          motor_spds_pwm[i] = 255;
          // Serial.println("speed maximised");
          // break;
        }

        if (motor_spds_pwm[i] < 1)
        {
          motor_spds_pwm[i] = 0;
          // Serial.println("speed mioinimised");
          // break;
        }

        prev_speed[i] = speed[i];
        prev_error[i] += error[i];
      }
      for (int i = 0; i < 4; i++)
      {
        motor_spds[i] = (int)speed[i];
      }
      its++;
      if (its > 30)
        break;
      if (xQueueReceive(speed_queue, &motor_s, xDelay))
      {
        // check if motor_s are all zero
        bool all_zero = true;
        for (int i = 0; i < 4; i++)
        {
          if (motor_s[i] != 0)
          {
            all_zero = false;
            break;
          }
        }
        if (all_zero)
        {
          // Serial.println("all motors stopped");
          robot_stop();
          emergency_stop = true;
          break; // exit the loop if all motors are stopped
        }
        setup_target_speed(&its, motor_s, target);
        // for (int i = 0; i < 4; i++)
        // {
        //   target[i] = (float)motor_s[i]; // set target speed
        // }
        // its = 0; // reset iterations
        // reset_encoders();
      }
    }
  }
}


void find_motor_speed(int *m_sp, int pwm_spd)
{
  unsigned long timer_strt;
  long cnts_prev[4];
  long cnts[4];

  for (int i = 0; i < 4; i++)
  {
    ledcWrite(pwm_channels[i], pwm_spd);
  }
  delay(200); // wait for 50ms to get encoder counts

  timer_strt = millis();
  reset_encoders();

  for (int i = 0; i < 4; i++)
  {
    cnts_prev[i] = get_encoder_pos(i);
  }
  while (millis() - timer_strt < 2000)
  {
    // wait for 50ms
  }
  for (int i = 0; i < 4; i++)
  {

    cnts[i] = get_encoder_pos(i);

    m_sp[i] = ((float)abs(cnts[i] - cnts_prev[i])) * pulse_dist * 1000 / ((float)millis() - timer_strt);
  }
  return;
}

// i need some code to dirve the seperate motors

#endif
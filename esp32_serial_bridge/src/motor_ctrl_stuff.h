

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

unsigned long max_motor_speed = 100; // max speed of motor in cms

volatile int motor_speed = 20;
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 350;
volatile unsigned long message_time = 100;

const int pwm_channels[] = {mtr_left_pwm_channel, mtr_left_pwm2_channel, mtr_right_pwm_channel, mtr_right_pwm2_channel};

int left_motor_speed;
int left_motor_speed2;
int right_motor_speed;
int right_motor_speed2;

bool update_speed = false; // if true, robot will update speed and send to other boards.

volatile int motor_spds[] = {20, 20, 20, 20};
volatile int motor_spds_pwm[] = {100, 100, 100, 100};

bool emergency_stop = false; // if true, robot will stop and not move until cleared.

enum state
{
  fwd = 0,
  rev,
  lft,
  rgt,
  fwdrgt,
  fwdlft,
  revrgt,
  revlft,
  stp,
  e_stop,
  e_stop_clear
};

// I should really set this as a private variable.
state actstate;
String str_status;
String status_names[11] = {"forward", "back", "left", "right", "forwardright", "forwardleft", "backright",
                           "backleft", "stop", "e_stop", "e_stop_clear"};

state st1 = stp;

// Placeholder for functions
void notifyClients();
void set_actstate(state s);
void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
void robot_move_loop();
void robot_set_speed(volatile int *ms);
uint8_t robo = 0;

int set_target_speed(float target, int motor, int init_spd);
void set_motor_speed_motor(uint8_t pwm_channel, uint8_t motor_dir, int spd);
void move_dist(void *params);
void move_theta(void *params);

int mtr_lft_state[] = {LEFT_FWD, LEFT_BACK, LEFT_BACK, LEFT_FWD, LEFT_FWD, LEFT_FWD, LEFT_BACK, LEFT_BACK}; // fwd, rev, lft, right
int mtr_rgt_state[] = {RIGHT_FWD, RIGHT_BACK, RIGHT_FWD, RIGHT_BACK, RIGHT_FWD, RIGHT_FWD, RIGHT_BACK, RIGHT_BACK};

void set_actstate(state s)
{
  int ret_val = 0;
  switch (s)
  {
  case e_stop_clear:
    /* code */
    if (actstate == e_stop)
    {
      actstate = e_stop_clear;
    }

    break;

  case e_stop:
    // Serial.println("setting e_stop");
    actstate = e_stop;
    robot_stop();
    emergency_stop = true;
    break;

  case stp:
    // Serial.println("setting stop");
    if (actstate == e_stop)
      break;
    actstate = stp;
    robot_stop();
    emergency_stop = true;
    break;

  default:
    if (actstate == e_stop)
      break;
    // set state
    actstate = s;
    emergency_stop = false;
    if (update_speed)
    {
      if (xQueueSend(speed_queue, &s, message_time) == pdFALSE)
      {
        Serial.println("Failed to send speed queue");
      }
    }
    else
    {

      if (xQueueSend(queue, &s, message_time))
      {
        // Serial.println("sending");
      } // portMAX_DELAY);
    }
    xQueueReceive(queue_ret, &ret_val, message_time); // hand shake
    // udpate other boards of changes to state.
    if (ret_val)
    {
      // myData.status = status_names[actstate];
      // xQueueSend(send_status_queu, &myData, message_time);
    }
    else
    {
      // robot_set_and_send_command(e_stop);
      actstate = e_stop;
      robot_stop();

      // myData.status = "e_stop";
      // xQueueSend(send_status_queu, &myData, message_time);
    }
    break;
  }
  notifyClients();
}

void robot_stop()
{
  // stop all motors
  emergency_stop = true;
  xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);

  for (int i = 0; i < 4; i++)
  {
    ledcWrite(pwm_channels[i], 0);
    // motor_spds_pwm[i] = 0;
  }

  xSemaphoreGive(Semaphore_pwm);
}

void robot_move_loop(void *parameter)
{

  for (;;)
  {
    state st;
    uint8_t rv = 1;
    // delay(1);

    xQueueReceive(queue, &st, portMAX_DELAY);
    // Serial.println("first order recieved: " + String((int)st));
    if ((int)st < (int)stp)
    {
      // Serial.println("order recieved");

      digitalWrite(LEFT_MOTOR_DIR, mtr_lft_state[(int)st]);
      digitalWrite(RIGHT_MOTOR_DIR, mtr_rgt_state[(int)st]);
      digitalWrite(LEFT_MOTOR_DIR2, mtr_lft_state[(int)st]);
      digitalWrite(RIGHT_MOTOR_DIR2, mtr_rgt_state[(int)st]);
      xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);
      if (!emergency_stop)
        for (int i = 0; i < 4; i++)
        {
          ledcWrite(pwm_channels[i], motor_spds_pwm[i]);
        } // setMotorSpeeds(motor_spds_pwm[0], motor_spds_pwm[2]);
      xSemaphoreGive(Semaphore_pwm);
    }
    else
    {
      robot_stop();
      emergency_stop = true;
    }

    xQueueSend(queue_ret, &rv, message_time);
    // if(st==e_stop){
    //     robot_stop();
    //     vTaskDelete(NULL);
    // }
  }
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

  dist_queue = xQueueCreate(7, sizeof(float));
  dist_queue_int = xQueueCreate(5, sizeof(bool));
  theta_queue = xQueueCreate(5, sizeof(float));
  theta_queue_int = xQueueCreate(5, sizeof(bool));
  speed_queue = xQueueCreate(5, sizeof(state));
  queue = xQueueCreate(5, sizeof(state));
  queue_ret = xQueueCreate(5, sizeof(uint8_t));

  Semaphore_pwm = xSemaphoreCreateMutex();
}

void robot_task_setup()
{

  xTaskCreatePinnedToCore(
      move_dist,
      "move_dist",
      5 * 1024,
      NULL,
      1,
      &move_dist_task,
      1);

  xTaskCreatePinnedToCore(
      move_theta,
      "move_theta",
      5 * 1024,
      NULL,
      1,
      &move_theta_task,
      1);

  xTaskCreatePinnedToCore(
      robot_move_loop, /* Function to implement the task */
      "robot_move",    /* Name of the task */
      4096,            /* Stack size in words */
      NULL,            /* Task input parameter */
      2,               /* Priority of the task */
      &robot_move,     /* Task handle. */
      1);
}

void robot_set_speed(volatile int *ms = motor_spds)
{
  // Serial.print("inside robot set speed: ");
  // Serial.println((int) actstate());
  int m_sp[4];

  if ((int)actstate < (int)stp)
  {
    if (((int)actstate > (int)rgt))
    {
      int turn_spd = 15;
      int ls;
      int rs;
      if (actstate == fwdlft || actstate == revlft)
      {
        ms[0] = ms[0] - turn_spd; // left motor speed
        ms[1] = ms[1] - turn_spd; // left motor speed 2
        ms[2] = ms[2] + turn_spd; // right motor speed
        ms[3] = ms[3] + turn_spd; // right motor speed 2
      }
      else
      {
        ms[0] = ms[0] + turn_spd; // left motor speed
        ms[1] = ms[1] + turn_spd; // left motor speed 2
        ms[2] = ms[2] - turn_spd; // right motor speed
        ms[3] = ms[3] - turn_spd; // right motor speed 2
      }
      for (int i = 0; i < 4; i++)
      {
        if (ms[i] < 0)
        {
          ms[i] = 0;
        }
        if (ms[i] > max_motor_speed)
        {
          ms[i] = max_motor_speed;
        }
      }
    }
    memcpy(m_sp, (int *)ms, sizeof(motor_spds));
    xQueueSend(speed_queue, &m_sp, message_time);
    for (int i = 0; i < 4; i++)
    {
      // ledcWrite(pwm_channels[i], ms[i]);
      // xQueueSend(speed_queue, &m_sp, message_time);
    }
    // setMotorSpeeds(0, 0);
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

void set_robot_target_speed(void *params)
{

  int motor_s[] = {10, 10, 10, 10};
  float dist = 0.0; // myparams->dist;
  bool stop;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  uint8_t rv = 1;

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

  float Kp = 2.0;
  float Kd = 0.5;
  float Ki = 0.1;

  float iterm = 0;
  float prev_error[4] = {0.0, 0.0, 0.0, 0.0}; // previous error for each motor
  float prev_speed[4];
  float pulse_dist = circum / ppr; // distance per pulse
  float target[4];                 // target speed in cm/s
  state st;

  while (true)
  {

    // Serial.println("in move dist");
    xQueueReceive(speed_queue, &st, portMAX_DELAY);
    if ((int)st < (int)stp && !emergency_stop)
    {
      // Serial.println("order recieved");

      digitalWrite(LEFT_MOTOR_DIR, mtr_lft_state[(int)st]);
      digitalWrite(RIGHT_MOTOR_DIR, mtr_rgt_state[(int)st]);
      digitalWrite(LEFT_MOTOR_DIR2, mtr_lft_state[(int)st]);
      digitalWrite(RIGHT_MOTOR_DIR2, mtr_rgt_state[(int)st]);
    }
    else
    {
      robot_stop();
      emergency_stop = true;
      // break;
    }
    xQueueSend(queue_ret, &rv, message_time);
    its = 0; // reset iterations
    reset_encoders();
    // memcpy(target, motor_s, sizeof(motor_s));

    for (int i = 0; i < 4; i++)
    {
      target[i] = (float)motor_spds[i]; // set target speed
      int abs_spd = abs(motor_spds[i]);
      if (abs_spd > 80)
        abs_spd = 80;
      motor_spds_pwm[i] = (int)((abs_spd / 80.0) * 255.0);
    }

    while (!emergency_stop && abs(speed[0] - target[0]) > tol &&
           abs(speed[1] - target[1]) > tol && abs(speed[2] - target[2]) > tol && abs(speed[3] - target[3]) > tol)
    {

      for (int i = 0; i < 4; i++)
      {
        cnts_prev[i] = get_encoder_pos(i);
        xSemaphoreTake(Semaphore_pwm, portMAX_DELAY);

        if (actstate < stp)
        {
          // need to inclulde a semaphore here to stop motors

          ledcWrite(pwm_channels[i], motor_spds_pwm[i]);
        }
        else
        {
          its = 20;                      // set iterations to 20 to stop motors
          ledcWrite(pwm_channels[i], 0); // stop motors if not moving
        }
        xSemaphoreGive(Semaphore_pwm);
        // update motor speed

        // set_motor_speed_motor((uint8_t)pwm_channel, dir_pin, init_spd);
      }
      timer_strt = millis();
      while (millis() - timer_strt < 50)
      {
        // wait for 50ms
      }
      for (int i = 0; i < 4; i++)
      {

        cnts[i] = get_encoder_pos(i);

        speed[i] = ((float)abs(cnts[i] - cnts_prev[i])) * pulse_dist * 1000 / ((float)millis() - timer_strt);
        if (its == 0)
          prev_speed[i] = speed[i]; // initialise previous speed

        error[i] = target[i] - speed[i];

        // iterm += Ki*error;
        iterm = Ki * prev_error[i]; // using momentum for the i-term instead.

        Serial.print(", error: ");
        Serial.println((int)((error[i]) * Kp + Kd * (speed[i] - prev_speed[i]) + iterm));
        motor_spds_pwm[i] += (int)(error[i] * Kp + Kd * (speed[i] - prev_speed[i]) + iterm);
        // init_spd -= (int) (Kd*(speed-prev_speed));
        if (cnts[i] == 0)
        {
          speed[i] = target[i]; // if counter is zero, there is an error in the encoder, so set speed to target speed
          int abs_spd = abs(motor_spds[i]);
          if (abs_spd > 80)
            abs_spd = 80;
          motor_spds_pwm[i] = (int)((abs_spd / 80.0) * 255.0);
        }

        if (motor_spds_pwm[i] > 255)
        {
          motor_spds_pwm[i] = 255;
          Serial.println("speed maximised");
          // break;
        }

        if (motor_spds_pwm[i] < 255)
        {
          motor_spds_pwm[i] = 0;
          Serial.println("speed miinimised");
          // break;
        }

        prev_speed[i] = speed[i];
        prev_error[i] = error[i];
      }
      for (int i = 0; i < 4; i++)
      {
        motor_spds[i] = speed[i];
      }
      its++;
      if (its > 10)
        break;
      if (xQueueReceive(speed_queue, &st, xDelay))
      {
        // resens st back to the queue
        xQueueSend(speed_queue, &st, message_time);
        break;
        // check if motor_s are all zero
        // bool all_zero = true;
        // for (int i = 0; i < 4; i++)
        // {
        //   if (motor_spds[i] != 0)
        //   {
        //     all_zero = false;
        //     break;
        //   }
        // }
        // if (all_zero)
        // {
        //   // Serial.println("all motors stopped");
        //   robot_stop();
        //   emergency_stop = true;
        //   break; // exit the loop if all motors are stopped
        // }
        // for (int i = 0; i < 4; i++)
        // {
        //   target[i] = (float)motor_spds[i]; // set target speed
        // }
        // its = 0; // reset iterations
        // reset_encoders();
      }
      if (emergency_stop)
      {
        // Serial.println("emergency stop");
        robot_stop();
        break; // exit the loop if emergency stop is triggered
      }
    }
  }
}

void calibrate_motors(float target_speed = 70)
{
  reset_encoders();

  // pid only use p

  // float traget_speed = 70;
  int point5_spd = 0;

  for (int i = 0; i < 4; i++)
  {
    point5_spd = set_target_speed(target_speed, i, 140);
    Serial.println("_________________________________");
    Serial.println("_________________________________");
    Serial.print("motor: ");
    Serial.print(i);
    Serial.print(", point5_spd: ");
    Serial.println(point5_spd);
    Serial.println("_________________________________");
    Serial.println("_________________________________");
  }

  // motor 1
  // start all motors
  // run for 5 seconds
  // output encoder values.
}

int set_target_speed(float target, int motor, int init_spd = 150)
{
  // pwm_pins
  // dir_pins
  unsigned long timer_strt;
  unsigned long timer_curr;

  int its = 0;

  uint8_t pwm_pin = pwm_pins[motor];
  uint8_t dir_pin = dir_pins[motor];

  const int pwm_channel = pwm_channels[motor];

  // run for 200ms, do encoder count

  float tol = 0.1;
  float error = 0.0;

  float speed = 0.0;

  long cnts_prev;
  long cnts;

  float Kp = 2.0;
  float Kd = 0.5;
  float Ki = 0.1;

  float iterm = 0;
  float prev_error = 0;
  float prev_speed;

  while (!emergency_stop && abs(speed - target) > tol)
  {
    timer_strt = millis();
    cnts_prev = get_encoder_pos(motor);

    set_motor_speed_motor((uint8_t)pwm_channel, dir_pin, init_spd);

    while (millis() - timer_strt < 50)
    {
    }

    cnts = get_encoder_pos(motor);

    speed = ((float)abs(cnts - cnts_prev)) * pulse_dist * 1000 / ((float)millis() - timer_strt);
    if (its == 0)
      prev_speed = speed; // initialise previous speed

    Serial.print("motor: ");
    Serial.print(motor);
    Serial.print(", speed: ");
    Serial.print(speed);
    Serial.print(", init_spd: ");
    Serial.print(init_spd);

    error = target - speed;

    // iterm += Ki*error;
    iterm = Ki * prev_error; // using momentum for the i-term instead.

    Serial.print(", error: ");
    Serial.println((int)((error)*Kp + Kd * (speed - prev_speed) + iterm));
    init_spd += (int)(error * Kp + Kd * (speed - prev_speed) + iterm);
    // init_spd -= (int) (Kd*(speed-prev_speed));

    if (init_spd > 255)
    {
      init_spd = 255;
      Serial.println("speed maximised");
      // break;
    }

    its++;
    if (its > 10)
      break;

    prev_speed = speed;
    prev_error = error;
  }

  set_motor_speed_motor((uint8_t)pwm_channel, dir_pin, 0);

  return init_spd;
}

void set_motor_speed_motor(uint8_t pwm_channel, uint8_t motor_dir, int spd)
{
  // for single motor
  bool fwd = true;

  if (spd < 0)
  {
    spd = -spd;
    fwd = false;
  }
  if (spd > 255)
    spd = 255;

  digitalWrite(motor_dir, fwd ? HIGH : LOW);
  ledcWrite(pwm_channel, spd);
}

void robot_fwd()
{
  if (actstate == e_stop)
  {
    robot_stop();
    return;
  }
  if (actstate != fwd)
  {
    set_actstate(fwd);
  }
}

void robot_back()
{
  if (actstate == e_stop)
  {
    robot_stop();
    return;
  }
  if (actstate != rev)
  {
    set_actstate(rev);
  }
}

void robot_left()
{
  if (actstate == e_stop)
  {
    robot_stop();
    return;
  }
  if (actstate != lft)
  {
    set_actstate(lft);
  }
}

void robot_right()
{
  if (actstate == e_stop)
  {
    robot_stop();
    return;
  }
  if (actstate != rgt)
  {
    set_actstate(rgt);
  }
}

double constrainAngle(double x)
{
  x = fmod(x + 180, 360);
  if (x < 0)
    x += 360;
  return x - 180;
}

void move_dist(void *params)
{

  float dist = 0.0; // myparams->dist;
  bool stop;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  while (true)
  {
    // Serial.println("in move dist");
    xQueueReceive(dist_queue, &dist, portMAX_DELAY);
    reset_encoders();

    if (dist > 0)
    {
      robot_fwd();
    }
    else if (dist < 0)
    {
      robot_back();
    }
    else
    {
      robot_stop();
      emergency_stop = true;
      Serial.println("dist is zero, stopping robot");
      continue;
    }

    Serial.print(dist);
    Serial.println(" distance recieved");

    // Serial.println("encoders reset");
    float total_pulses = abs(dist / pulse_dist);
    Serial.println(total_pulses);
    unsigned long t_strt = millis();
    long enc_puls_prev[] = {0, 0, 0, 0};

    long cm30 = (long)(30.0 / pulse_dist); // number of pulses in 30cm
    int spd0;
    while (!emergency_stop && ((float)(abs(encoder0Pos) + abs(encoder2Pos) + abs(encoder1Pos) + abs(encoder3Pos))) < 4 * total_pulses)
    {

      if ((millis() - t_strt > 30))
      {
        t_strt = millis();

        for (int i = 1; i < 4; i++)
        {
          int ms;
          ms = abs(motor_spds_pwm[i]);
          if (ms == 0)
            break;
          if (abs(get_encoder_pos(i) - enc_puls_prev[i]) > abs(encoder0Pos - enc_puls_prev[0]))
          {
            motor_spds_pwm[i]--;
          }
          else if (abs(get_encoder_pos(i) - enc_puls_prev[i]) < abs(encoder0Pos - enc_puls_prev[0]))
          {
            motor_spds_pwm[i]++;
          }

          enc_puls_prev[i] = get_encoder_pos(i);
          // if (abs(enc_puls_prev[0]) > cm30 ){
          //   if (spd0 > 100) motor_spds[i] -=5;
          //   motor_spds[i] -= 5;
          //   if (motor_spds[i]<25){
          //     motor_spds[i] = 25;
          //   }
          // }
        }
        enc_puls_prev[0] = get_encoder_pos(0);
        if ((total_pulses - abs(enc_puls_prev[0])) < cm30)
        {
          spd0 = motor_spds_pwm[0];
          // 30 cm left to travel
          if (spd0 > 55)
          {
            for (int j = 0; j < 4; j++)
            {
              motor_spds_pwm[j] = motor_spds_pwm[j] - 3;
            }
          }
        }
      }

      // if (xQueueReceive(dist_queue_int, &stop, xDelay)) {
      //   dist=0.0;
      //   break;
      // }

      if (xQueueReceive(dist_queue, &dist, xDelay))
      {
        // keep going for that distance
        total_pulses = abs(dist / pulse_dist);
        reset_encoders();
        for (int i = 0; i < 4; i++)
        {
          enc_puls_prev[i] = get_encoder_pos(i);
        }
        Serial.println("interrupting cow!");
      }
      // vTaskDelay(500);
      // Serial.println(total_pulses);
      // Serial.println(encoder0Pos);
    }
    robot_stop();
    emergency_stop = true;
    Serial.println(encoder0Pos);
    Serial.println(encoder2Pos);
    // now reset motor speeds
  }
}

void move_theta(void *params)
{

  float thet = 0.0; // myparams->dist;
  bool stop;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  float dist = 0.0;
  float total_pulses = 0.0;

  while (true)
  {
    // Serial.println("in move dist");
    xQueueReceive(theta_queue, &thet, portMAX_DELAY);
    reset_encoders();
    dist = turning_circle * (thet / 360);
    total_pulses = abs(dist / pulse_dist);

    while (!emergency_stop && ((float)(abs(encoder0Pos) + abs(encoder2Pos))) < 2 * total_pulses)
    {
      if (dist > 0)
      {
        robot_right();
      }
      if (dist < 0)
      {
        robot_left();
      }
      // if (xQueueReceive(dist_queue_int, &stop, xDelay)) {
      //   dist=0.0;
      //   break;
      // }

      if (xQueueReceive(theta_queue, &thet, xDelay))
      {
        // keep going for that distance
        dist = turning_circle * (thet / 360);
        total_pulses = abs(dist / pulse_dist);
        reset_encoders();
        Serial.println("interrupting cow!");
      }
      // vTaskDelay(500);
      // Serial.println(total_pulses);
      // Serial.println(encoder0Pos);
    }
    robot_stop();
    emergency_stop = true;
    // vTaskDelete(NULL);
  }
}

void move_robot_dist(float dist)
{
  reset_encoders();
  Serial.println(dist);
  float total_pulses = abs(dist / pulse_dist);
  Serial.println(total_pulses);
  while (!emergency_stop && abs(encoder0Pos) + abs(encoder2Pos) < 2 * total_pulses)
  {
    if (dist > 0)
    {
      robot_fwd();
    }
    if (dist < 0)
    {
      robot_back();
    }
    delay(200);
    Serial.println(encoder0Pos);
  }
  robot_stop();
}

void turn_robot(float thet)
{
  reset_encoders();
  //   thet = constrainAngle(thet);
  float dist = turning_circle * (thet * 2 * pi / 360);
  float total_pulses = abs(dist / pulse_dist);
  while (!emergency_stop && abs(encoder0Pos) + abs(encoder2Pos) < 2 * total_pulses)
  {
    if (thet > 0)
    {
      robot_right();
    }
    if (thet < 0)
    {
      robot_left();
    }
  }
  robot_stop();
}

// i need some code to dirve the seperate motors

#endif
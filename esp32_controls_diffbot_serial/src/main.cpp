#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "encoders_stuff.h"
#include "esp_now_stuff.h"
#include "motor_ctrl_stuff.h"
#include "serial_stuff.h"
#include "OLED_stuff.h"



// put function declarations here:
void esp32_task_setup();

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  init_encoders();
  robot_setup();

 

  unsigned long timer_strt;
  unsigned long timer_cur;
  timer_strt = millis();
  timer_cur = timer_strt;

  bool calib=false;
  if (calib){
    int ms[4] = {0, 0, 0, 0};
    int pwm;
    for (pwm = 0; pwm < 256; pwm += 15)
    {
      find_motor_speed(ms, pwm);
      Serial.print("pwm: ");
      Serial.print(pwm);
      Serial.print(", motor speeds: ");
      for (int i = 0; i < 4; i++)
      {
        Serial.print(ms[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    // Serial.print("pwm: 255");
    // find_motor_speed(ms,255);
    // Serial.println("Motor speeds:");
    // for (int i = 0; i < 4; i++)
    // {
    //   Serial.print("Motor: ");
    //   Serial.print(i);
    //   Serial.print(", speed: ");
    //   Serial.println(ms[i]);
    // }

  } //if calib
  while (false){

    for (int j = 0; j < 4; j++)
    {
      
  for (int i = 0; i < 4; i++)
  {
    int cnts; 
    // reset_encoders();
    ledcWrite(pwm_channels[i],170); // motor_spds_pwm[i]);
    delay(1000);
    cnts = get_encoder_pos(j);
    Serial.print("counts for motor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(j);
    Serial.print(": ");
    Serial.println(cnts);
    ledcWrite(pwm_channels[i],0);
    delay(500);

  }
}
}

  emergency_stop=true;
  // move_robot_dist(2000.0);

  
  esp32_task_setup();
  robot_stop();

}


void esp32_task_setup(){

  xTaskCreatePinnedToCore(
      set_robot_target_speed,
      "set_robot_target_speed",
      5 * 1024,
      NULL,
      1,
      &set_robot_target_speed_task,
      1);

  xTaskCreatePinnedToCore(
      serial_command_process,
      "serial_command_process",
      5 * 1024,
      NULL,
      1,
      &serial_command_process_task,
      1);

  
}

// i need a function that when it recieves a serial command, it calls a function and returns a string


void loop() {
  vTaskDelete(NULL); // delete the loop task, as we are using FreeRTOS tasks instead of the main loop
  // put your main code here, to run repeatedly:
}


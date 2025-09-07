#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "html_stuff_new.h"
#include "encoders_stuff.h"
#include "esp_now_stuff.h"
#include "motor_ctrl_stuff.h"
#include "OLED_stuff.h"
#include "ssid_stuff.h"


// put function declarations here:


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);


  init_msj();
 
  initSPIFFS();

  initWiFi();

  initWebSocket();

  Serial.println("WebSocket initialized");

  init_html();

  Serial.println("HTML initialized");

  init_encoders();
  robot_setup();

  left_motor_speed=106;
  right_motor_speed=106;

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
  emergency_stop=true;
  // move_robot_dist(2000.0);

  
  robot_task_setup();
  robot_stop();

}

void loop() {
  vTaskDelete(NULL); // delete the loop task, as we are using FreeRTOS tasks instead of the main loop
  // put your main code here, to run repeatedly:
}



#ifndef HTML_STUFF_H_
#define HTML_STUFF_H_

// Import required libraries
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "ssid_stuff.h"
#include "motor_ctrl_stuff.h"
#include "encoders_stuff.h"
#include "esp_now_stuff.h"
#include "OLED_stuff.h"

IPAddress local_IP(192, 168, 1, 211);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

AsyncWebServer server(80);
// Create a WebSocket object
AsyncWebSocket ws("/ws");

unsigned long last_update;
unsigned long ws_clean;
unsigned long clean_up_every = 10000;

TaskHandle_t clean_up_ws;
TaskHandle_t stp_robot_moving;
TaskHandle_t debug_loop_task;

SemaphoreHandle_t Semaphore_ws;

// set up json value
bool em_stop_int = false;
bool em_stop_cleared = false;

// enum state {fwd=0, rev=1, lft=2, rgt=3, stp=4,e_stop,e_stop_clear};

JSONVar motor_status_json;

float goal_dist;

void IRAM_ATTR emergency_stop_interrupt()
{
  em_stop_int = true;
  // actstate = e_stop;

  // notifyClients();
}

void IRAM_ATTR clear_emergency_stop()
{
  em_stop_cleared = true;
  // delay(5);
  // robot_set_and_send_command(e_stop_clear);
  // actstate = e_stop_clear;
  // set_actstate(e_stop_clear);
  // notifyClients();
}

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_AP_STA); // need this mode to use esp now

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Go to: http://");
  Serial.println(WiFi.localIP());
}

// motor status json
void init_msj()
{
  motor_status_json["forward"] = ((int)fwd);
  motor_status_json["back"] = ((int)rev);
  motor_status_json["left"] = ((int)lft);
  motor_status_json["right"] = ((int)rgt);
  motor_status_json["forwardleft"] = ((int)fwdlft);
  motor_status_json["backleft"] = ((int)revlft);
  motor_status_json["forwardright"] = ((int)fwdrgt);
  motor_status_json["backright"] = ((int)revrgt);
  motor_status_json["stop"] = ((int)stp);
  motor_status_json["e_stop"] = ((int)e_stop);
  motor_status_json["e_stop_clear"] = ((int)e_stop_clear);
}

void notifyClients()
{
  str_status = status_names[(int)actstate];
  String status = "{\"speed\":" + String(motor_speed) + ",\"status\":" + "\"" + str_status + "\"" + ",\"goal_dist\":" + "\"" + goal_dist + "\"}";
  // Serial.println(status);
  // ws.cleanupClients();
  ws.textAll(status);
}

// mostly cut and pasted from random nerd tutorials.
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    // Serial.println((char *)data);
    JSONVar received_object = JSON.parse((char *)data);
    const char *action = received_object["action"];
    if (strcmp(action, "speed") == 0)
    {
      update_speed = true; // set to true so we update speed
      last_update = millis();
      // Serial.println(received_object["value"]);
      motor_speed = (int)atoi(received_object["value"]);
      for (int i = 0; i < 4; i++)
      {
        motor_spds[i] = motor_speed;
      }
      // motor_speed = map(motor_speed, 0, 100, 0,255)
      // Serial.println(motor_speed);
      // robot_set_speed();
      notifyClients();
    }
    if (strcmp(action, "pwm") == 0)
    {
      // Serial.println(received_object["value"]);
      update_speed = false; // set to false so we don't update speed
      last_update = millis();
      motor_speed = (int)atoi(received_object["value"]);
      for (int i = 0; i < 4; i++)
      {
        motor_spds_pwm[i] = motor_speed;
      }
      // motor_speed = map(motor_speed, 0, 100, 0,255)
      // Serial.println(motor_speed);
      // robot_set_speed();
      notifyClients();
    }
    if (strcmp(action, "goal_dist") == 0)
    {
      // Serial.println(received_object["value"]);
      goal_dist = (int)atoi(received_object["value"]);

      notifyClients();
    }
    if (strcmp(action, "status") == 0)
    {
      last_update = millis();
      const char *status = received_object["value"];
      str_status = String(status);
      // Serial.println(str_status);
      if (motor_status_json.hasOwnProperty(status))
      {
        int stmp = motor_status_json[status];
        set_actstate((state)stmp);
        str_status = status_names[(int)actstate];
        // notifyClients();
        // if (millis() - ws_clean > (clean_up_every))
        // {
        //   ws.cleanupClients();
        //   ws_clean = millis();
        // }
      }
      else
      {
        // do nothing
      }

      // Serial.println(stmp);
    }
    if (strcmp(action, "theta") == 0)
    {
      Serial.println("theta sreceived");
      last_update = millis();
      float theta = (int)atof(received_object["value"]);

      // Serial.println(stmp);
    }
    if (strcmp(action, "dist") == 0)
    {
      Serial.println("theta sreceived");
      last_update = millis();
      float dist = (int)atof(received_object["value"]);

      // Serial.println(stmp);
    }
    if (strcmp(action, "open") == 0)
    {
      // const char *status = received_object["value"];
      str_status = status_names[(int)actstate];
      ws.cleanupClients(); // everytime a new connection is opened.

      notifyClients();
    }
    if (strcmp(action, "update") == 0)
    {
      // const char *status = received_object["value"];
      // if it has been more than 90ms since last recieved don't do anything
      if (millis() - last_update > (message_time - 10))
      {
        // then ew update
        last_update = millis();
        const char *status = received_object["value"];

        str_status = String(status);

        if (motor_status_json.hasOwnProperty(status))
        {
          xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
          previous_time = millis();
          xSemaphoreGive(Semaphore_prev_time);

          // int stmp = motor_status_json[status];
          // robot_set_and_send_command((state)stmp);
          // str_status = status_names[(int)actstate];
        }
        else
        {
          // do nothing
        }
      }
    }
  }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}
void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void clean_ws(void *parameters)
{
  for (;;)
  {
    vTaskDelay(100000 / portTICK_PERIOD_MS);
    xSemaphoreTake(Semaphore_ws, portMAX_DELAY);
    ws.cleanupClients();
    xSemaphoreGive(Semaphore_ws);
  }
}

// if we haven't recieved any commands for 500ms, then we will stop the robot. This is because if we set a command on a website
// and then get disconnected, it will not keep running.
void call_stp(void *parameters)
{
  unsigned long pt;
  for (;;)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
    pt = previous_time;
    xSemaphoreGive(Semaphore_prev_time);

    if (millis() - pt > 2000)
    {
      // Serial.println(millis()-pt);
      set_actstate(stp);
      // Serial.println("Stopping");
    }
  }
}

void debug_loop(void *parameters)
{

  for (;;)
  {
    vTaskDelay(500 / portTICK_PERIOD_MS);

    if (em_stop_cleared)
    {
      if (actstate == e_stop)
        set_actstate(e_stop_clear);
      // notifyClients();
      em_stop_cleared = false;
      Serial.println("in em stop clear");
    }

    if (em_stop_int)
    {
      notifyClients();
      em_stop_int = false;
      set_actstate(e_stop);
      Serial.println("in em stop int");
    }

    // Serial.print("left: ");
    // Serial.println(encoder1Pos);
    // Serial.println(encoder0Pos);
    if (false)
      if ((float)abs(encoder1Pos) > one_m)
      {
        last_update = millis();

        str_status = "stop";
        Serial.println(str_status);
        // int stmp = motor_status_json["stop"];
        // Serial.println(stmp);
        set_actstate((state)stp);
        encoder1Pos = 0;
        // notifyClients();//this gets called in set and set command
      }
  }
}

void init_html()
{
  // put your setup code here, to run once:

  // robot_pos = {0.0, 0.0, 0.0}; // pi / 2};
  // target_pos = {0.0, 300.0, pi / 2};
  // theta = robot_pos.theta;

  // delay(2000);

  // esp_now_setup();

  // pinMode(EM_STOP_INT_CLEAR_PIN, INPUT_PULLDOWN);
  // pinMode(EM_STOP_INT_PIN, INPUT_PULLDOWN);

  // attachInterrupt(EM_STOP_INT_PIN, emergency_stop, RISING);
  // attachInterrupt(EM_STOP_INT_CLEAR_PIN, clear_emergency_stop, RISING);

  // set_actstate(e_stop);//start in emergency stop. So we need a clear stop before we can move.

  // register_peers();

  // init_encoders();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.on("/goal_seek", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index_goal.html", "text/html"); });

  server.on("/manual_drive", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

  // robot_setup();

  previous_time = millis();
  last_update = millis();

  Semaphore_prev_time = xSemaphoreCreateMutex();
  Semaphore_ws = xSemaphoreCreateMutex();

  // xTaskCreatePinnedToCore(
  //     clean_ws,      /* Function to implement the task */
  //     "clean_up_ws", /* Name of the task */
  //     1000,          /* Stack size in words */
  //     NULL,          /* Task input parameter */
  //     0,             /* Priority of the task */
  //     &clean_up_ws,  /* Task handle. */
  //     1);            /* Core where the task should run */

  if (false)
    xTaskCreatePinnedToCore(
        call_stp,          /* Function to implement the task */
        "call_stp",        /* Name of the task */
        15360,             /* Stack size in words */
        NULL,              /* Task input parameter */
        0,                 /* Priority of the task */
        &stp_robot_moving, /* Task handle. */
        1);                /* Core where the task should run */
  if (false)
  xTaskCreatePinnedToCore( // set true if you want to debug
      debug_loop,          /* Function to implement the task */
      "debug_loop",        /* Name of the task */
      2000,                /* Stack size in words */
      NULL,                /* Task input parameter */
      0,                   /* Priority of the task */
      &debug_loop_task,    /* Task handle. */
      1);

  // init_encoders();

  notifyClients(); // emergency stop
  // oled.clearDisplay();
  // oled.setFont(&FreeSans9pt7b);
  // oled.setTextColor(WHITE);
  // oled.setTextSize(1);
  // oled.setCursor(0, 17);

  // oled.println(WiFi.localIP());
  // oled.display();
}

#endif // HTML_STUFF_H_
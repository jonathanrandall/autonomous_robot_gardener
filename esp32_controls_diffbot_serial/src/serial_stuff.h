#ifndef SERIAL_STUFF_H_
#define SERIAL_STUFF_H_

#include <Arduino.h>
#include "motor_ctrl_stuff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Motor speed array to store the 4 values
int motor_s[4];
float pid[3] = {0.1, 0.01, 0.005}; // default PID values

// Function to parse motor speed command "m val1 val2 val3 val4"
bool parse_command_d(String cmd, int *ms){
  sscanf(cmd.c_str(), "m_%d_%d_%d_%d", &ms[0], &ms[1], &ms[2], &ms[3]);
  return true;
}
bool parse_command_ints(String cmd, int *ms, int max_values = 5)
{
  String tokens[max_values];
  int count = 0;

  int start = 0;
  while (start < cmd.length() && count < max_values)
  {
    // Skip leading spaces
    while (start < cmd.length() && cmd[start] == ' ')
      start++;

    if (start >= cmd.length())
      break;

    // Find end of token
    int end = cmd.indexOf(' ', start);
    if (end == -1)
      end = cmd.length();

    // Extract token
    tokens[count++] = cmd.substring(start, end);

    start = end + 1;
  }

  // Must have exactly 5 tokens: "m", val1, val2, val3, val4
  if (count != max_values)
    return false;

  if (tokens[0] != "m")
    return false;

  // Validate & convert to int
  for (int i = 0; i < (max_values - 1); i++)
  {
    String t = tokens[i + 1];
    for (unsigned int j = 0; j < t.length(); j++)
    {
      if (!isDigit(t[j]) && !(j == 0 && t[j] == '-'))
      {
        return false; // invalid number
      }
    }
    ms[i] = t.toInt();

    //clip between 0 and 82
    ms[i] = constrain(ms[i], 0, 82);

  }

  if (false)
    for (int i = 0; i < (max_values - 1); i++)
    {
      Serial.println(ms[i]);
    }

  return true;
}

bool parse_command_floats(String cmd, float *pid_, int max_values = 4)
{
  String tokens[max_values];
  int count = 0;

  int start = 0;
  while (start < cmd.length() && count < max_values)
  {
    // Skip leading spaces
    while (start < cmd.length() && cmd[start] == ' ')
      start++;

    if (start >= cmd.length())
      break;

    // Find end of token
    int end = cmd.indexOf(' ', start);
    if (end == -1)
      end = cmd.length();

    // Extract token
    tokens[count++] = cmd.substring(start, end);

    start = end + 1;
  }

  // Must have exactly 5 tokens: "m", val1, val2, val3, val4
  if (count != max_values)
    return false;

  if (tokens[0] != "u")
    return false;

  // Validate & convert to float
  for (int i = 0; i < (max_values - 1); i++)
  {
    String t = tokens[i + 1];
    bool valid = false;
    // Accept numbers like -1.23, 4.56, etc.
    if (t.length() > 0 && (isDigit(t[0]) || t[0] == '-' || t[0] == '+'))
    {
      valid = true;
      for (unsigned int j = 1; j < t.length(); j++)
      {
        if (!isDigit(t[j]) && t[j] != '.' && t[j] != 'e' && t[j] != 'E' && t[j] != '-' && t[j] != '+')
        {
          valid = false;
          break;
        }
      }
    }
    if (!valid)
      return false;
    pid_[i] = t.toFloat();
  }

  return true;
}

// Function to handle motor speed commands and send to queue
void handleMotorCommand(String cmd)
{
  if (parse_command_d(cmd, motor_s))
  {
    if (false)
      for (int i = 0; i < (4); i++)
      {
        Serial.println(motor_s[i]);
      }
    // Send motor speeds to queue
    // check if speed_queue is full
    if (uxQueueMessagesWaiting(speed_queue) == 0)
    {
      // i want to wait 30ms max to send to queue 
      
      xQueueSend(speed_queue, &motor_s, 30 / portTICK_PERIOD_MS);
      // Serial.println("Motor speeds queued successfully\r");
    }
    else
    {
      Serial.println("Speed queue is full, command ignored\r");
    }
  }
  else
  {
    // Serial.println("Invalid motor command format. Use: m val1 val2 val3 val4 " + cmd + "_");
  }
}

void handlePIDCommand(String cmd)
{
  if (parse_command_floats(cmd, pid))
  {
    Kp = pid[0];
    Ki = pid[1];
    Kd = pid[2];
    // Serial.println("Updated PID values: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd) + "\r");
  }
  else
  {
    // Serial.println("Invalid motor command format. Use: u val1 val2 val3\r");
  }
}

String handleSerialCommand(String cmd)
{
  // Example: parse command and call functions
  if (cmd == "status")
  {
    // Call a status function and return its result
    return "Robot status: OK";
  }
  else if (cmd == "stop")
  {
    robot_stop();
    return "Robot stopped";
  }
  else
  {
    return "Unknown command_";
  }
}

void serial_command_process(void *pvParameters)
{
  // QueueHandle_t speed_queue = (QueueHandle_t)pvParameters;
  String cmd;
  int tmp = 0; // terminate for strings

  for (;;)
  {
    if (Serial.available())
    {
      cmd = Serial.readStringUntil('\r');
      cmd.trim(); // Remove any whitespace

      if (cmd.startsWith("m_"))
      {
        // Handle motor speed command
        handleMotorCommand(cmd);
        // Serial.println("Motor command received_");
        // Serial.print("\r\n");
      }
      else if (cmd.startsWith("u"))
      {
        handlePIDCommand(cmd);
        // Serial.println("PID command received\r");
      }
      else if (cmd.startsWith("s"))
      {
        robot_stop();
        // Serial.println("robot stop command received\r");
      }
      else if (cmd.startsWith("e"))
      {
        // Serial.println("e command recieved_\r");
        // String response = String(motor_spds[0]) + " " + String(motor_spds[1]) + " " + String(motor_spds[2]) + " " + String(motor_spds[3]) + "_";
        // Serial.println(response);
        // Serial.print("_\r\n");
      }

      else
      {
        // Handle other commands
        String response = handleSerialCommand(cmd);
        // Serial.println(response);
      }
      String response = String(motor_spds[0]) + " " + String(motor_spds[1]) + " " + String(motor_spds[2]) + " " + String(motor_spds[3]) + " " + String(tmp);
      Serial.println(response);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield to other tasks
  }
}

#endif
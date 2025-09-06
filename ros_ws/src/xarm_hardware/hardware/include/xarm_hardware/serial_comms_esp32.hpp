#ifndef TRAJECTORY_ESP32_COMMS_HPP
#define TRAJECTORY_ESP32_COMMS_HPP

#pragma once

// #include <sstream>
#include <sstream>
#include <iostream>
#include <libserial/SerialPort.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <vector>
#include <libserial/SerialStream.h>
#include <rclcpp/rclcpp.hpp>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class SerialComms
{
public:
  SerialComms() = default;
  ~SerialComms() { stop(); }

  bool init_serial(const std::string &port, int baud_rate, double timeout)
  {
    try
    {
      timeout_ms_ = static_cast<int>(timeout * 1000);
      timeout_ms_ = std::min(50, timeout_ms_); // minimum 50ms timeout
      serial_port_ = std::make_unique<LibSerial::SerialPort>();
      serial_port_->Open(port);
      convert_baud_rate(baud_rate);

      serial_port_->SetBaudRate((LibSerial::BaudRate::BAUD_115200));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      serial_port_->Write("bus_servo.set_positions([500,350,500,300,500,500],1000)\r\n");
      previous_positions_ = {500.0, 350.0, 500.0, 500.0, 500.0, 500.0};
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      running_ = true;
      do_update_ = true;
      write_position();
      // std::thread t(&SerialComms::serialLoop, this);
      //   t.detach(); // "fire-and-forget"
      // serial_thread_ = std::thread(&SerialComms::serialLoop, this);

      // assign latest_positions_ and latest_velocities_ to default values
      latest_positions_.assign(6, 0.0);   // = std::vector<double>(6, 0.0);
      previous_positions_.assign(6, 500.0); 
      latest_velocities_.assign(6, 0.0);  // = std::vector<double>(6, 0.0);
      latest_commands_.assign(6, 500.0);  // = std::vector<double>(6, 0.0);
      previous_commands_.assign(6, -1.0); // = std::vector<double>(6, 0.0);

      return true;
    }
    catch (const LibSerial::OpenFailed &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
                   "Failed to open serial port %s: %s", port.c_str(), e.what());
      running_ = false;
      return false;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
                   "Serial init failed: : %s", e.what());

      // std::cerr << "Serial init failed: " << e.what() << std::endl;
      return false;
    }
  }

  bool is_running()
  {
    return running_;
  }

  bool get_do_update()
  {
    return do_update_;
  }

  void stop()
  {
    running_ = false;

    if (serial_port_ && serial_port_->IsOpen())
    {
      serial_port_->Close();
      serial_port_.reset();
    }
  }

  // Called by ros2_control::read()
  void get_latest_state(std::vector<double> &positions,
                        std::vector<double> &velocities)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    positions = latest_positions_;
    velocities = latest_velocities_;
  }

  std::vector<double> get_joint_positions()
  {
    std::vector<double> positions;
    positions.resize(6);
    std::lock_guard<std::mutex> lock(data_mutex_);
    positions = latest_positions_;
    return positions;
  }

  std::vector<double> read_joint_position()
  {
    //i don't really need to call this function because i call read inside write_position
    if (!serial_port_ || !serial_port_->IsOpen())
      return std::vector<double>(6, 0.0);
    // else return latest_commands_;

    try
    {
      // --- Request positions ---
      serial_port_->FlushIOBuffers();
      serial_port_->Write("bus_servo.get_positions()\r\n");

      std::string response;
      auto start_time = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(100))
      {
        // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
        //               "in while loop");
        if (serial_port_->IsDataAvailable())
        {
          // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
          //     "in if statement");
          std::string chunk;
          try
          {
            serial_port_->Read(chunk, 1); // ReadLine(chunk, ']', timeout_ms_  );
          }
          catch (const LibSerial::ReadTimeout &e)
          {
            RCLCPP_WARN(rclcpp::get_logger("SerialComms"),
                        "Read timeout: %s", e.what());
          }

          response += chunk;

          size_t start = response.find('[');
          size_t end = response.find(']');
          if (start != std::string::npos && end != std::string::npos && end > start)
          {
            // parseState(response);
            break;
          }
        }
        else
        {
          // break;
        }

      }

      RCLCPP_DEBUG(rclcpp::get_logger("SerialComms"),
                  "Read from ESP32: %s", response.c_str());

      if (!response.empty())
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        parseState(response);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
                   "Serial read error: %s", e.what());
    }
    return latest_positions_;

  }

  void write_position()
  {
    // std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_port_ || !serial_port_->IsOpen())
      return;
    // if (!do_update_)
    //   return;
    // return;

    try
    {
      // --- Request positions ---
      if (do_update_)
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (!latest_commands_.empty())
        {
          std::string cmd = makeCommandString(latest_commands_);
          serial_port_->Write(cmd);
          RCLCPP_DEBUG(rclcpp::get_logger("SerialComms"),
                      "Wrote to ESP32: %s", cmd.c_str());
          previous_commands_ = latest_commands_;
          do_update_ = false;
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration_ms_));
        }
      }

      // return;

      serial_port_->FlushIOBuffers();
      serial_port_->Write("bus_servo.get_positions()\r\n");

      std::string response;
      auto start_time = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(100))
      {
        // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
        //               "in while loop");
        if (serial_port_->IsDataAvailable())
        {
          // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
          //     "in if statement");
          std::string chunk;
          try
          {
            serial_port_->Read(chunk, 1); // ReadLine(chunk, ']', timeout_ms_  );
          }
          catch (const LibSerial::ReadTimeout &e)
          {
            RCLCPP_WARN(rclcpp::get_logger("SerialComms"),
                        "Read timeout: %s", e.what());
          }

          // chunk.resize(256);
          // chunk = "tmp";
          // serial_port_->Read(chunk, 256);
          // serial_port_->ReadLine(chunk, '\n', timeout_ms_); // read available bytes
          response += chunk;

          // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
          //             "Read from ESP32: %s", response.c_str());

          size_t start = response.find('[');
          size_t end = response.find(']');
          if (start != std::string::npos && end != std::string::npos && end > start)
          {
            // parseState(response);
            break;
          }

        }
        else
        {
          // break;
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // RCLCPP_INFO(rclcpp::get_logger("SerialComms"),
        //               "out if statement");
      }

      RCLCPP_DEBUG(rclcpp::get_logger("SerialComms"),
                  "Read from ESP32: %s", response.c_str());

      if (!response.empty())
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        parseState(response);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
                   "Serial read error: %s", e.what());
    }
  }

  // Called by ros2_control::write()
  bool set_latest_command(const std::vector<double> &commands)
  {
    {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    RCLCPP_DEBUG(rclcpp::get_logger("XArmHardware"),
                 "Commands are  positions for ESP32: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                 commands[0], commands[1], commands[2],
                 commands[3], commands[4], commands[5]);
    latest_commands_ = commands;
    }
    do_update_ = false;
    for (size_t i = 0; i < latest_commands_.size(); ++i)
    {
      // Only send if command has changed
      if (latest_commands_[i] != previous_commands_[i] || abs(latest_positions_[i] - latest_commands_[i]) > 5.0)
      {
        do_update_ = true;
        write_position();
        // if (serial_mutex_.try_lock()){
        //   do_update_ = true;
        // } else {
        //   do_update_ = false;
        // }
        //  serial_mutex_.unlock();
        break;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("XArmHardware"),
                 "Commands are  positions latest for ESP32: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                 latest_commands_[0], latest_commands_[1], latest_commands_[2],
                 latest_commands_[3], latest_commands_[4], latest_commands_[5]);

    return true;
  }

private:

  bool parseState(const std::string &response)
  {
    // TODO: parse your robot’s protocol
    // Example: just push dummy data for now

    // latest_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    latest_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    try
    {
      // Look for position data in the response
      // Expected format: [pos1, pos2, pos3, pos4, pos5, pos6]
      size_t start = response.find('[');
      size_t end = response.find(']');

      if (start == std::string::npos || end == std::string::npos || end <= start)
      {
        RCLCPP_WARN(rclcpp::get_logger("SerialComms"),
                    "Invalid position response format: %s", response.c_str());
        return false;
      }

      std::string positions_str = response.substr(start + 1, end - start - 1);
      std::istringstream iss(positions_str);
      std::string token;

      size_t i = 0;
      while (std::getline(iss, token, ',') && i < latest_positions_.size())
      {
        // Remove whitespace and convert to double
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);

        if (!token.empty())
        {
          latest_positions_[i] = std::stod(token);
          // latest_positions_[i] = 0.6*latest_positions_[i] + 0.4*previous_positions_[i]; // scale to 0-1000
          // if (i==2) latest_positions_[i] = 400.0;
          i++;

        }
      }
      RCLCPP_DEBUG(rclcpp::get_logger("SerialComms"),
                  "position response format: %s", response.c_str());
      // latest_positions_[2] = 0.0;

      return i == latest_positions_.size();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
                   "Failed to parse positions: %s", e.what());
      return false;
    }
  }

  std::string makeCommandString(const std::vector<double> &cmds)
  {
    // TODO: encode your servo command protocol
    // Example: just return a placeholder string
    std::ostringstream oss;
    oss << "bus_servo.set_positions([";
    for (size_t i = 0; i < cmds.size(); ++i)
    {
      if (i > 0)
        oss << ", ";
      oss << static_cast<int>(cmds[i]); // Convert to integer for servo positions
    }
    oss << "],"<< sleep_duration_ms_ << ")\r\n"; // 1000ms execution time
    return oss.str();     //"bus_servo.set_positions([...], 1000)\r\n";
  }

private:
  std::unique_ptr<LibSerial::SerialPort> serial_port_;

  std::thread serial_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> do_update_{true};

  std::mutex data_mutex_;
  std::mutex cmd_mutex_;
  std::mutex serial_mutex_;

  std::vector<double> latest_positions_;
  std::vector<double> previous_positions_;
  std::vector<double> latest_velocities_;
  std::vector<double> latest_commands_;
  std::vector<double> previous_commands_;
  int timeout_ms_{100};
  int sleep_duration_ms_{500};
};

#endif //
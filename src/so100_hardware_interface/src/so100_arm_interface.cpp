// Copyright (c) 2025, Sumit Patidar
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_so100_interface/so100_arm_interface.hpp"

namespace {
constexpr char kLoggerName[] = "so100_hardware_interface";

bool string_to_bool(const std::string &value) {
  return value == "1" || value == "true" || value == "True" || value == "TRUE";
}

} // namespace

namespace so100_hardware_interface {

hardware_interface::CallbackReturn
SO100ArmInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  hw_states_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  joint_ids_.clear();
  joint_ids_.reserve(joint_count);

  for (const auto &joint : info_.joints) {
    const auto id_it = joint.parameters.find("id");
    if (id_it == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Joint '%s' is missing required 'id' parameter.",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    try {
      joint_ids_.push_back(std::stoi(id_it->second));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Failed to parse joint id '%s' for joint '%s': %s",
                   id_it->second.c_str(), joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  use_serial_ = false;
  const auto use_serial_it = info_.hardware_parameters.find("use_serial");
  if (use_serial_it != info_.hardware_parameters.end()) {
    use_serial_ = string_to_bool(use_serial_it->second);
  }

  if (use_serial_) {
    const auto port_it = info_.hardware_parameters.find("serial_port");
    if (port_it == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "Parameter 'serial_port' is required when 'use_serial' is true.");
      return CallbackReturn::ERROR;
    }
    serial_port_ = port_it->second;

    const auto baud_it = info_.hardware_parameters.find("serial_baudrate");
    if (baud_it == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "Parameter 'serial_baudrate' is required when 'use_serial' is true.");
      return CallbackReturn::ERROR;
    }
    try {
      baud_rate_ = std::stoi(baud_it->second);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Failed to parse 'serial_baudrate' value '%s': %s",
                   baud_it->second.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  } else {
    serial_port_.clear();
    baud_rate_ = 0;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO100ArmInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!use_serial_) {
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Serial communication disabled via 'use_serial'; skipping "
                "hardware checks.");
    return CallbackReturn::SUCCESS;
  }

  if (serial_connected_) {
    sts3215_.end();
    serial_connected_ = false;
  }

  if (!sts3215_.begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Failed to establish serial connection on port '%s' with baud %d.",
        serial_port_.c_str(), baud_rate_);
    return CallbackReturn::ERROR;
  }
  serial_connected_ = true;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    int servo_id = joint_ids_[i];
    // Check if the servo responds to Ping
    if (sts3215_.Ping(servo_id) != -1) {
      RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Found motor with ID: %d",
                  servo_id);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Motor with ID %d did not respond to Ping.", servo_id);
      return CallbackReturn::ERROR;
    }
    // Read initial position and match with hw_states_
    if (sts3215_.FeedBack(servo_id) != -1) {
      int pos = sts3215_.ReadPos(-1); // -1 to read from cached feedback data
      hw_states_[i] = bitsToRads(pos, i);
      hw_commands_[i] = hw_states_[i];
      RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                  "Motor ID %d initial position: %.2f radians (%.2f degress)",
                  servo_id, hw_states_[i], radsToDegs(hw_states_[i]));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
              "All %zu motors responded successfully.", info_.joints.size());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SO100ArmInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name,
                                  hardware_interface::HW_IF_POSITION,
                                  &hw_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SO100ArmInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name,
                                    hardware_interface::HW_IF_POSITION,
                                    &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SO100ArmInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
              "Activating SO100ArmInterface...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO100ArmInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (serial_connected_) {
    // First disable torque on all servos while connection is still active
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      if (!sts3215_.EnableTorque(joint_ids_[i], 0)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                    "Failed to disable torque for motor ID %d", joint_ids_[i]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Then close the serial connection
    sts3215_.end();
    serial_connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Servos disabled and serial connection closed.");
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SO100ArmInterface::read(const rclcpp::Time & /*time*/,
                        const rclcpp::Duration & /*period*/) {
  if (serial_connected_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      int servo_id = joint_ids_[i];
      if (sts3215_.FeedBack(servo_id) != -1) {
        int pos = sts3215_.ReadPos(servo_id);
        hw_states_[i] = bitsToRads(pos, i);
      } else {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                    "Failed to read position for motor with ID %d", servo_id);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SO100ArmInterface::write(const rclcpp::Time & /*time*/,
                         const rclcpp::Duration & /*period*/) {
  if (serial_connected_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      int servo_id = joint_ids_[i];
      int target_pos_bits = radsToBits(hw_commands_[i], i);
      if (!sts3215_.RegWritePosEx(servo_id, target_pos_bits, 2400, 50)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                    "Failed to write position for motor with ID %d", servo_id);
      }
    }
    sts3215_.RegWriteAction();
  }
  return hardware_interface::return_type::OK;
}

double SO100ArmInterface::bitsToRads(int bits, int joint_index) {
  // Assuming 0-4095 bits correspond to -180 to +180 degrees
  double radians =
      (static_cast<double>(bits - calibration_offsets_[joint_index]) *
       direction_factors_[joint_index] * M_PI) /
      2048.0;
  return radians;
}

int SO100ArmInterface::radsToBits(double rads, int joint_index) {
  // Assuming -180 to +180 degrees correspond to 0-4095 bits
  int bits = static_cast<int>((rads * 2048.0) /
                              (M_PI)*direction_factors_[joint_index]) +
             calibration_offsets_[joint_index];
  // Clamp bits to valid range
  if (bits < 0) {
    bits = 0;
  } else if (bits > 4095) {
    bits = 4095;
  }
  return bits;
}

double SO100ArmInterface::radsToDegs(double rads) {
  return rads * (180.0 / M_PI);
}

double SO100ArmInterface::bitsToRadPerSec(int speed_bits, int max_bits,
                                          double max_rpm) {
  // Clamp input for safety
  if (speed_bits < 0)
    speed_bits = 0;
  if (speed_bits > max_bits)
    speed_bits = max_bits;

  // Convert bits → RPM
  double rpm = (static_cast<double>(speed_bits) / max_bits) * max_rpm;

  // Convert RPM → rad/s
  double rad_per_sec = rpm * 2.0 * M_PI / 60.0;

  return rad_per_sec;
}
} // namespace so100_hardware_interface

PLUGINLIB_EXPORT_CLASS(so100_hardware_interface::SO100ArmInterface,
                       hardware_interface::SystemInterface)

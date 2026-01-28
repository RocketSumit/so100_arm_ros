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

#ifndef SO100_HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_
#define SO100_HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <ftservo/SCServo.h>

namespace so100_hardware_interface {
class SO100ArmInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SO100ArmInterface)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  // std::unordered_map<std::string, joint_limits::JointLimits> joint_limits_;
  std::vector<int> joint_ids_;

  // Serial communication
  bool use_serial_{false};
  std::string serial_port_;
  int baud_rate_{0};
  bool serial_connected_{false};

  // Calibration zero offsets
  std::vector<int> calibration_offsets_{2048, 2048, 2048, 2048, 2048, 2048};
  std::vector<int> direction_factors_{1, 1, 1, 1, 1, 1};

  // Servo interface
  SMS_STS sts3215_;

  // Convert bit reading to radians
  double bitsToRads(int bits, int joint_index);
  // Convert radians to bit writing
  int radsToBits(double rads, int joint_index);
  // Convert radians to degrees
  double radsToDegs(double rads);
  // Convert degrees to radians
  double bitsToRadPerSec(int speed_bits, int max_bits = 2400,
                         double max_rpm = 45.0);
};

} // namespace so100_hardware_interface

#endif // SO100_HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_

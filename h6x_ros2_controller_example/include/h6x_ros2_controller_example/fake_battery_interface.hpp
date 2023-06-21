// Copyright 2023 HarvestX Inc.
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

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <limits>

#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace h6x_ros2_controller_example
{
class FakeBatteryInterface : public hardware_interface::SensorInterface
{
private:
  using CallbackReturn = hardware_interface::CallbackReturn;

  std::vector<double> hw_states_;

  const std::array<std::string, 6> state_interface_names = {
    "voltage", "temperature", "current", "charge", "capacity", "percentage"
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FakeBatteryInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
};
}  // namespace h6x_ros2_controller_example

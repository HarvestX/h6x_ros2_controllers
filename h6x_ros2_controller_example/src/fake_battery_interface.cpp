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

#include "h6x_ros2_controller_example/fake_battery_interface.hpp"

namespace h6x_ros2_controller_example
{
FakeBatteryInterface::CallbackReturn FakeBatteryInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(
    info_.sensors[0].state_interfaces.size(),
    std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

FakeBatteryInterface::CallbackReturn FakeBatteryInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
  }

  return CallbackReturn::SUCCESS;
}

FakeBatteryInterface::CallbackReturn FakeBatteryInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeBatteryInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[0].name, state_interface_names[i], &hw_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::return_type FakeBatteryInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // read battery data
  hw_states_[0] = 3.3;
  hw_states_[1] = 24.4;
  hw_states_[2] = 0.4;
  hw_states_[3] = 24;
  hw_states_[4] = 4.7;
  hw_states_[5] = 0.6;

  return hardware_interface::return_type::OK;
}

}  // namespace h6x_ros2_controller_example

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  h6x_ros2_controller_example::FakeBatteryInterface,
  hardware_interface::SensorInterface)

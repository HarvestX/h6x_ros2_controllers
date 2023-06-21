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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace semantic_components
{
class BatteryState : public SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatteryState(const std::string & name)
  : SemanticComponentInterface(name, 1)
  {
    this->interface_names_.emplace_back(this->name_ + "/" + "voltage");
    this->interface_names_.emplace_back(this->name_ + "/" + "temperature");
    this->interface_names_.emplace_back(this->name_ + "/" + "current");
    this->interface_names_.emplace_back(this->name_ + "/" + "charge");
    this->interface_names_.emplace_back(this->name_ + "/" + "capacity");
    this->interface_names_.emplace_back(this->name_ + "/" + "percentage");
  }

  virtual ~BatteryState() = default;

  bool get_value_as_message(sensor_msgs::msg::BatteryState & message)
  {
    message.voltage = state_interfaces_[0].get().get_value();
    message.temperature = state_interfaces_[1].get().get_value();
    message.current = state_interfaces_[2].get().get_value();
    message.charge = state_interfaces_[3].get().get_value();
    message.capacity = state_interfaces_[4].get().get_value();
    message.percentage = state_interfaces_[5].get().get_value();

    return false;
  }
};
}  // namespace semantic_components

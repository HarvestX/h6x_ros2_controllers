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
#include <vector>

#include "controller_interface/controller_interface.hpp"
// auto-generated by generate_parameter_library
#include "image_sensor_broadcaster_parameters.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/image.hpp"

namespace image_sensor_broadcaster
{
class ImageSensorBroadcaster : public controller_interface::ControllerInterface
{
protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> parameter_name_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;

private:
  std::array<std::string, 6>
  interface_name_ = {"height", "width", "encoding", "is_bigendian", "step"};

public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
};
}  // namespace image_sensor_broadcaster

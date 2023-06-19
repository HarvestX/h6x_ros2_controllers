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

#include "image_sensor_broadcaster/image_sensor_broadcaster.hpp"

namespace image_sensor_broadcaster
{
controller_interface::CallbackReturn ImageSensorBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();

  try {
    // register ft sensor data publisher
    this->sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Image>("~/image", rclcpp::SensorDataQoS());
    this->realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();

  realtime_publisher_->msg_.header.frame_id = this->params_.frame_id;
  realtime_publisher_->msg_.encoding = this->params_.encoding;
  realtime_publisher_->msg_.is_bigendian = static_cast<uint8_t>(this->params_.is_bigendian);
  realtime_publisher_->msg_.height = static_cast<uint32_t>(this->params_.height);
  realtime_publisher_->msg_.width = static_cast<uint32_t>(this->params_.width);
  realtime_publisher_->msg_.step =
    static_cast<uint32_t>(this->params_.width *
    sensor_msgs::image_encodings::numChannels(this->params_.encoding));

  realtime_publisher_->msg_.data.reserve(
    realtime_publisher_->msg_.step * realtime_publisher_->msg_.height);

  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ImageSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ImageSensorBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return state_interfaces_config;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "activating");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ImageSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    realtime_publisher_->msg_.header.set__stamp(time);
    // realtime_publisher_->msg_.set__data();
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
}  // namespace image_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  image_sensor_broadcaster::ImageSensorBroadcaster, controller_interface::ControllerInterface)

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
    this->param_listener_ = std::make_shared<ParamListener>(this->get_node());
    this->params_ = this->param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  this->params_ = this->param_listener_->get_params();

  this->image_sensor_ = std::make_unique<semantic_components::ImageSensor>(
    semantic_components::ImageSensor(this->params_.sensor_name));

  try {
    // register ft sensor data publisher
    this->sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Image>("~/image", rclcpp::SensorDataQoS());
    this->realtime_publisher_ = std::make_unique<StatePublisher>(this->sensor_state_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  this->realtime_publisher_->lock();

  this->realtime_publisher_->msg_.header.frame_id = this->params_.frame_id;
  this->realtime_publisher_->msg_.encoding = this->params_.encoding;
  this->realtime_publisher_->msg_.is_bigendian = static_cast<uint8_t>(this->params_.is_bigendian);
  this->realtime_publisher_->msg_.height = static_cast<uint32_t>(this->params_.height);
  this->realtime_publisher_->msg_.width = static_cast<uint32_t>(this->params_.width);
  this->realtime_publisher_->msg_.step =
    static_cast<uint32_t>(this->params_.width *
    sensor_msgs::image_encodings::numChannels(this->params_.encoding));

  this->realtime_publisher_->msg_.data.resize(
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
  state_interfaces_config.names = this->image_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->image_sensor_->assign_loaned_state_interfaces(this->state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImageSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->image_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ImageSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (this->realtime_publisher_ && this->realtime_publisher_->trylock()) {
    this->realtime_publisher_->msg_.header.set__stamp(time);
    this->image_sensor_->get_value_as_message(this->realtime_publisher_->msg_);
    this->realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
}  // namespace image_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  image_sensor_broadcaster::ImageSensorBroadcaster, controller_interface::ControllerInterface)

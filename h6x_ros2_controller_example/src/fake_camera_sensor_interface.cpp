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

#include "h6x_ros2_controller_example/fake_camera_sensor_interface.hpp"

namespace h6x_ros2_controller_example
{

FakeCameraSensorInterface::~FakeCameraSensorInterface()
{
  boost::interprocess::shared_memory_object::remove(this->shm_->get_name());
  this->shm_.reset();
}

FakeCameraSensorInterface::CallbackReturn FakeCameraSensorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  for (const auto & sensor : info.sensors) {
    for (const auto & state_interface : sensor.state_interfaces) {
      if (state_interface.name == "image") {
        this->shm_key_ = sensor.name + "-" + state_interface.name;
      }
    }
  }

  if (this->shm_key_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "FakeCameraSensorInterface"), "sensor with 'image' state interface does not exist");
    return CallbackReturn::ERROR;
  }

  // TODO(anyone) : Check hardware information
  this->info_ = info;

  // Yield sample data
  const std::array<uint8_t, 3> O = {0, 0, 0};
  const std::array<uint8_t, 3> R = {255, 0, 0};
  const std::array<uint8_t, 3> B = {0, 0, 255};

  this->data_.clear();
  for (const auto & elm : {O, O, O, R, R, O, B, B, O, R, R, O, O, O}) {
    for (const auto & val : elm) {
      this->data_.emplace_back(val);
    }
  }

  this->shm_ = std::make_unique<boost::interprocess::shared_memory_object>(
    boost::interprocess::open_or_create, this->shm_key_.c_str(), boost::interprocess::read_write);

  this->shm_->truncate(this->data_.size());

  return CallbackReturn::SUCCESS;
}

FakeCameraSensorInterface::CallbackReturn FakeCameraSensorInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  this->map_ = std::make_unique<boost::interprocess::mapped_region>(
    *this->shm_, boost::interprocess::read_write);

  return CallbackReturn::SUCCESS;
}

FakeCameraSensorInterface::CallbackReturn FakeCameraSensorInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  this->map_.reset();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeCameraSensorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_if;
  state_if.emplace_back(
    hardware_interface::StateInterface(
      this->info_.sensors.at(0).name, "image", &this->state_if_val_));
  return state_if;
}

hardware_interface::return_type FakeCameraSensorInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // memcpy(image_mem_ptr, (void *)data.data(), data.size());

  if (this->map_) {
    this->state_if_val_ = 0.0;
    std::memcpy(
      this->map_->get_address(), this->data_.data(), this->map_->get_size());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace h6x_ros2_controller_example

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  h6x_ros2_controller_example::FakeCameraSensorInterface,
  hardware_interface::SensorInterface)

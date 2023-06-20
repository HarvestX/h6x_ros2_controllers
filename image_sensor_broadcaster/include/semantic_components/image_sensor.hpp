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

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace semantic_components
{
class ImageSensor : public SemanticComponentInterface<sensor_msgs::msg::Image>
{
private:
  std::string shm_key_;
  std::shared_ptr<boost::interprocess::shared_memory_object> shm_;
  std::shared_ptr<boost::interprocess::mapped_region> map_;

public:
  explicit ImageSensor(const std::string & name)
  : SemanticComponentInterface(name, 1)
  {
    this->interface_names_.emplace_back(this->name_ + "/" + "image");

    this->shm_key_ = this->name_ + "-" + "image";
    this->shm_ = std::make_shared<boost::interprocess::shared_memory_object>(
      boost::interprocess::open_only, this->shm_key_.c_str(), boost::interprocess::read_only);

    this->map_ = std::make_shared<boost::interprocess::mapped_region>(
      *this->shm_, boost::interprocess::read_only);
  }

  virtual ~ImageSensor() = default;

  bool get_value_as_message(sensor_msgs::msg::Image & message)
  {
    if (!std::isnan(this->state_interfaces_[0].get().get_value()) && this->map_) {
      std::memcpy(message.data.data(), this->map_->get_address(), this->map_->get_size());
      return true;
    }

    return false;
  }
};
}  // namespace semantic_components

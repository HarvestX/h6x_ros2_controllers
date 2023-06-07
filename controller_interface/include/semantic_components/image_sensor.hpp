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

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace semantic_components
{

class ImageSensor : public SemanticComponentInterface<sensor_msgs::msg::Image>
{
public:
  explicit ImageSensor(const std::string & name)
  : SemanticComponentInterface(name, 10)
  {
    interface_names_.emplace_back(name_ + "/" + "height");
    interface_names_.emplace_back(name_ + "/" + "width");
    interface_names_.emplace_back(name_ + "/" + "encoding");
    interface_names_.emplace_back(name_ + "/" + "is_bigendian");
    interface_names_.emplace_back(name_ + "/" + "step");
    interface_names_.emplace_back(name_ + "/" + "data");
  }

  /// Return height.
  /**
   * Return height reported by an Image
   *
   * \return vector with height values.
   */
  uint32_t get_height()
  {
    size_t interface_offset = 0;
    height_ = state_interfaces_[interface_offset].get().get_value();
    return height_;
  }

  /// Return width.
  /**
   * Return width reported by an Image
   *
   * \return vector with width values.
   */
  uint32_t get_width()
  {
    size_t interface_offset = 1;
    width_ = state_interfaces_[interface_offset].get().get_value();
    return width_;
  }

  /// Return encoding.
  /**
   * Return encoding reported by an Image
   *
   * \return vector with encoding values.
   */
  std::string get_encoding()
  {
    size_t interface_offset = 2;
    encoding_ = state_interfaces_[interface_offset].get().get_value();
    return encoding_;
  }

  /// Return is_bigendian.
  /**
   * Return is_bigendian reported by an Image
   *
   * \return vector with is_bigendian values.
   */
  uint8_t get_is_bigendian()
  {
    size_t interface_offset = 3;
    is_bigendian_ = state_interfaces_[interface_offset].get().get_value();
    return is_bigendian_;
  }

  /// Return step.
  /**
   * Return step reported by an Image
   *
   * \return vector with step values.
   */
  uint32_t get_step()
  {
    size_t interface_offset = 4;
    step_ = state_interfaces_[interface_offset].get().get_value();
    return step_;
  }

  /// Return data.
  /**
   * Return data reported by an Image
   *
   * \return vector with data values.
   */
  std::vector<uint8_t> get_data()
  {
    size_t interface_offset = 5;
    size_t data_size = step_ * height_;
    for (size_t i = 0; i < data_size; i++) {
      data_[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return data_;
  }

  /// Return Image message with data
  /**
   * Constructs and return a Image message from the current values.
   * \return image message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Image & message)
  {
    // call get_*() to update with the latest values
    get_height();
    get_width();
    get_encoding();
    get_is_bigendian();
    get_step();
    get_data();

    // update the message values, covariances unknown
    message.height = height_;
    message.width = width_;
    message.encoding = encoding_;
    message.is_bigendian = is_bigendian_;
    message.step = step_;

    size_t data_size = step_ * height_;
    for (size_t i = 0; i < data_size; i++) {
      message.data[i] = data_[i];
    }

    return true;
  }

protected:
  uint32_t height_;
  uint32_t width_;
  std::string encoding_;
  uint8_t is_bigendian_;
  uint32_t step_;
  std::vector<uint8_t> data_;
};
}  // namespace semantic_components

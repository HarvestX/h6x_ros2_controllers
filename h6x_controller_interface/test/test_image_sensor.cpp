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

#include "test_image_sensor.hpp"

#include <memory>
#include <string>
#include <vector>

void ImageSensorTest::TearDown()
{
  image_sensor_.reset(nullptr);
}

TEST_F(ImageSensorTest, validate_all)
{
  // create the Image sensor
  image_sensor_ = std::make_unique<TestableImageSensor>(sensor_name_);

  // validate the component name
  ASSERT_EQ(image_sensor_->name_, sensor_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(image_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(image_sensor_->state_interfaces_.capacity(), size_);

  // validate the default interface_names_
  ASSERT_TRUE(
    std::equal(
      image_sensor_->interface_names_.begin(), image_sensor_->interface_names_.end(),
      full_interface_names_.begin(), full_interface_names_.end()));

  // get the interface names
  std::vector<std::string> interface_names = image_sensor_->get_state_interface_names();

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(10);

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(angular_velocity_y);
  temp_state_interfaces.emplace_back(orientation_y);
  temp_state_interfaces.emplace_back(linear_acceleration_y);
  temp_state_interfaces.emplace_back(orientation_x);
  temp_state_interfaces.emplace_back(linear_acceleration_z);
  temp_state_interfaces.emplace_back(angular_velocity_z);
  temp_state_interfaces.emplace_back(orientation_z);
  temp_state_interfaces.emplace_back(orientation_w);
  temp_state_interfaces.emplace_back(angular_velocity_x);
  temp_state_interfaces.emplace_back(linear_acceleration_x);

  // now call the function to make them in order like interface_names
  image_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(image_sensor_->state_interfaces_.size(), size_);

  // validate get_values_as_message
  sensor_msgs::msg::Image temp_message;
  ASSERT_TRUE(image_sensor_->get_values_as_message(temp_message));

  // release the state_interfaces_
  image_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(image_sensor_->state_interfaces_.size(), 0u);
}

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

#include "gmock/gmock.h"

#include "semantic_components/image_sensor.hpp"

// implementing and friending so we can access member variables
class TestableImageSensor : public semantic_components::
  ImageSensor
{
  FRIEND_TEST(ImageSensorTest, validate_all);

public:
  // Use generation of interface names
  explicit TestableImageSensor(const std::string & name)
  : ImageSensor(name)
  {}

  virtual ~TestableImageSensor() = default;
};

class ImageSensorTest : public ::testing::Test
{
public:
  void SetUp()
  {
    full_interface_names_.reserve(size_);
    for (auto index = 0u; index < size_; ++index) {
      full_interface_names_.emplace_back(sensor_name_ + "/" + image_interface_names_[index]);
    }
  }

  void TearDown();

protected:
  const size_t size_ = 10;
  const std::string sensor_name_ = "test_Image";
  std::unique_ptr<TestableImageSensor> image_sensor_;

  std::vector<std::string> full_interface_names_;
  const std::vector<std::string> image_interface_names_ =
  {"height", "width", "encoding", "is_bigendian", "step", "data"};
};

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
#include <memory>
#include <string>

#include <gmock/gmock.h>

#include <image_sensor_broadcaster/image_sensor_broadcaster.hpp>

class FriendImageSensorBroadcaster
  : public image_sensor_broadcaster::ImageSensorBroadcaster
{
  FRIEND_TEST(ImageSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(ImageSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(ImageSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(ImageSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(ImageSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(ImageSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(ImageSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(ImageSensorBroadcasterTest, SensorStatePublishTest);
};


class ImageSensorBroadcasterTest : public ::testing::Test
{
protected:
  const std::string sensor_name_ = "img_sensor";
  const std::string frame_id_ = "img_sensor_frame";
  const std::string encoding_ = "rgb8";
  const int height_ = 1, width_ = 16;

  double image_value_ = std::numeric_limits<double>::quiet_NaN();
  std::unique_ptr<FriendImageSensorBroadcaster> img_broadcaster_;

  hardware_interface::StateInterface image_data_ {
    this->sensor_name_, "image", &this->image_value_};

  std::string shm_key_;
  std::unique_ptr<boost::interprocess::shared_memory_object> shm_;
  std::unique_ptr<boost::interprocess::mapped_region> map_;

public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpImageBroadcaster();

  void subscribe_and_get_message(sensor_msgs::msg::Image &);
};

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

#include "test_image_sensor_broadcaster.hpp"

#include <vector>
#include <memory>
#include <utility>

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace


void ImageSensorBroadcasterTest::SetUpTestCase() {}

void ImageSensorBroadcasterTest::TearDownTestCase() {}

void ImageSensorBroadcasterTest::SetUp()
{
  this->img_broadcaster_ = std::make_unique<FriendImageSensorBroadcaster>();
  this->shm_ = std::make_unique<boost::interprocess::shared_memory_object>(
    boost::interprocess::open_or_create, (this->sensor_name_ + "-" + "image").c_str(),
    boost::interprocess::read_write);

  this->shm_->truncate(
    this->height_ * this->width_ * sensor_msgs::image_encodings::numChannels(this->encoding_));
  this->map_ = std::make_unique<boost::interprocess::mapped_region>(
    *this->shm_, boost::interprocess::read_write);
}

void ImageSensorBroadcasterTest::TearDown()
{
  this->img_broadcaster_.reset(nullptr);
  this->map_.reset(nullptr);
  boost::interprocess::shared_memory_object::remove(this->shm_->get_name());
  this->shm_.reset(nullptr);
}


void ImageSensorBroadcasterTest::SetUpImageBroadcaster()
{
  const auto result = this->img_broadcaster_->init("test_image_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<hardware_interface::LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->image_data_);

  this->img_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}


void ImageSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Image & img_msg)
{
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Image::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Image>(
    "/test_image_sensor_broadcaster/image", rclcpp::SensorDataQoS(), subs_callback);

  int max_sub_check_loop_count = 5;
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  while (max_sub_check_loop_count--) {
    this->img_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready) {
      break;
    }
  }

  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
    "controller/broadcaster update loop";

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(img_msg, msg_info));
}

TEST_F(ImageSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpImageBroadcaster();

  this->img_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->img_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});
  this->img_broadcaster_->get_node()->set_parameter({"encoding", this->encoding_});
  this->img_broadcaster_->get_node()->set_parameter({"height", this->height_});
  this->img_broadcaster_->get_node()->set_parameter({"width", this->width_});

  ASSERT_EQ(this->img_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ImageSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpImageBroadcaster();

  this->img_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->img_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});
  this->img_broadcaster_->get_node()->set_parameter({"encoding", this->encoding_});
  this->img_broadcaster_->get_node()->set_parameter({"height", this->height_});
  this->img_broadcaster_->get_node()->set_parameter({"width", this->width_});

  ASSERT_EQ(this->img_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->img_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ImageSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpImageBroadcaster();

  this->img_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->img_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});
  this->img_broadcaster_->get_node()->set_parameter({"encoding", this->encoding_});
  this->img_broadcaster_->get_node()->set_parameter({"height", this->height_});
  this->img_broadcaster_->get_node()->set_parameter({"width", this->width_});

  ASSERT_EQ(this->img_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->img_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    this->img_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(ImageSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpImageBroadcaster();

  this->img_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->img_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});
  this->img_broadcaster_->get_node()->set_parameter({"encoding", this->encoding_});
  this->img_broadcaster_->get_node()->set_parameter({"height", this->height_});
  this->img_broadcaster_->get_node()->set_parameter({"width", this->width_});

  ASSERT_EQ(this->img_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->img_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::Image img_msg;
  subscribe_and_get_message(img_msg);

  EXPECT_EQ(img_msg.header.frame_id, this->frame_id_);
  EXPECT_EQ(img_msg.encoding, this->encoding_);
  ASSERT_EQ(img_msg.height, static_cast<uint32_t>(this->height_));
  ASSERT_EQ(img_msg.width, static_cast<uint32_t>(this->width_));
  ASSERT_EQ(
    img_msg.step, static_cast<uint32_t>(
      this->width_ * sensor_msgs::image_encodings::numChannels(this->encoding_)));

  printf("Data size %lu", img_msg.data.size());
  // ASSERT_FALSE(true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

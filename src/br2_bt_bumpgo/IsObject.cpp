// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <utility>

#include "br2_bt_bumpgo/IsObject.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_bt_bumpgo
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsRedBall::IsRedBall(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  detection2D_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "output_detection_2d", 100, std::bind(&IsRedBall::detection_callback_, this, _1));
}

void IsRedBall::detection_callback_(vision_msgs::msg::Detection2DArray::UniquePtr msg)
{
  last_detection2D_ = std::move(msg);
  // last_time_ = last_detection2D_->header.stamp;
}

BT::NodeStatus
IsRedBall::tick()
{
  if (last_detection2D_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }

  auto elapsed = node_->now() - rclcpp::Time(last_detection2D_->header.stamp);

  if (elapsed > 1s) {
    std::cout << "No veo pelota." << std::endl;
    return BT::NodeStatus::FAILURE;
  } else {
    std::cout << "Veo pelota." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_bumpgo::IsRedBall>("IsObject");
}

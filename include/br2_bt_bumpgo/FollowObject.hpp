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

#ifndef BR2_BT_BUMPGO__FOLLOWOBJECT_HPP_
#define BR2_BT_BUMPGO__FOLLOWOBJECT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "br2_bt_bumpgo/PIDController.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_bt_bumpgo
{

class FollowBall : public BT::ActionNodeBase
{
public:
  explicit FollowBall(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

  void detection_callback_(vision_msgs::msg::Detection3DArray::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3D_sub_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_detection3D_;

  PIDController spin_pid_;
  PIDController forward_pid_;
};

}  // namespace br2_bt_bumpgo

#endif  // BR2_BT_BUMPGO__FOLLOWOBJECT_HPP_

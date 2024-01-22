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
#include <iostream>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "br2_bt_bumpgo/FollowObject.hpp"
#include "br2_bt_bumpgo/PIDController.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace br2_bt_bumpgo
{

using namespace std::chrono_literals;
using std::placeholders::_1;

FollowBall::FollowBall(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  spin_pid_(-1.0, 1.0, 0.0, 1.0),
  forward_pid_(-1.0, 1.0, 0.0, 1.0)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);

  detection3D_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", 100, std::bind(&FollowBall::detection_callback_, this, _1));
}

void
FollowBall::halt()
{
}

BT::NodeStatus
FollowBall::tick()
{
  if (last_detection3D_ == nullptr) {
    std::cout << "nullptr" << std::endl;
    return BT::NodeStatus::RUNNING;
  }
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  auto & x_3D = last_detection3D_->detections[0].bbox.center.position.x;
  auto & y_3D = last_detection3D_->detections[0].bbox.center.position.y;
  auto & z_3D = last_detection3D_->detections[0].bbox.center.position.z;

  auto dist = sqrt((x_3D * x_3D) + (y_3D * y_3D) + (z_3D * z_3D));

  std::cout << "DISTANCE: " << dist << std::endl;

  spin_pid_.set_pid(0.8, 0.05, 0.55);
  forward_pid_.set_pid(0.8, 0.05, 0.55);

  geometry_msgs::msg::Twist vel_msgs;

  double angle = atan2(y_3D, x_3D);
  double angle_deg = std::round(angle * 180.0 / M_PI);  // Radians to degrees.

  auto normalized_ang = (-angle_deg - 90) / 90;

  std::cout << "ANGLE: " << normalized_ang << std::endl;

  vel_msgs.angular.z = std::clamp(forward_pid_.get_output(normalized_ang), -0.5, 0.5);
  vel_msgs.linear.x = std::clamp(spin_pid_.get_output(dist - 1), -0.3, 0.3);

  std::cout << "VEL: " << vel_msgs.linear.x << std::endl;
  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;  // Always return RUNNING to restart de RS.
}


void FollowBall::detection_callback_(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  last_detection3D_ = std::move(msg);
}

}  // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_bumpgo::FollowBall>("FollowObject");
}

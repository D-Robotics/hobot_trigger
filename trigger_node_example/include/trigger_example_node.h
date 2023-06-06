// Copyright (c) 2023，Horizon Robotics.
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

#ifndef trigger_node_example_H_
#define trigger_node_example_H_

#include <ctime>
#include <iostream>

#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"

#include "trigger_node/trigger_node.h"

using rclcpp::NodeOptions;

using hobot::trigger_node::TriggerNode;

class TriggerExampleNode : public TriggerNode {
 public:

  TriggerExampleNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  ~TriggerExampleNode() override;

  // 测试
  int Gtest();

  // 判断是否启动Trigger模块，并控制订阅器
  int IsStart() override;

  void EventTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  std::string event_msg_sub_topic_name_ = "/ai_msg_mono2d_trash_detection";
 
 private:

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      event_msg_subscription_ = nullptr;

};

#endif  // trigger_node_example_H_
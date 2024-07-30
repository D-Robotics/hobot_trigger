// Copyright (c) 2024，D-Robotics.
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

#include "include/trigger_example_node.h"

TriggerExampleNode::TriggerExampleNode(const std::string &node_name, const NodeOptions &options)
    : TriggerNode(node_name, options) {
  RCLCPP_WARN(rclcpp::get_logger("example"), "TriggerExampleNode Init.");
}

TriggerExampleNode::~TriggerExampleNode() {}

int TriggerExampleNode::IsStart() {

  if (config_.status == 0) {
    event_msg_subscription_ == nullptr;
    return 0;
  } 

  if (event_msg_subscription_ != nullptr) {
    return 0;
  }

  event_msg_subscription_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        event_msg_sub_topic_name_,
        10,
        std::bind(&TriggerExampleNode::EventTopicCallback,
                  this,
                  std::placeholders::_1));
  return 0;
}

void TriggerExampleNode::EventTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  std::stringstream ss;
  ss << "Recved msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
     << ", targets size: " << msg->targets.size();

  int threshold = 0;
  switch (config_.trigger_type) {
    case 1110: threshold = 3; break;
    case 1111: threshold = 4; break;
  }

  if (msg->targets.size() >= static_cast<size_t>(threshold)) {

    auto now = std::chrono::system_clock::now();
    auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // request gtest
    Config request;
    request = config_;
    request.timestamp = milliseconds_since_epoch;

    if (last_request_.timestamp == 0) {
      last_request_ = request;
      requests_.push(request);
      RCLCPP_WARN(rclcpp::get_logger("example"), "Trigger Event!");
      return;
    }
    
    if ((request.timestamp - request.duration_ts_front) <= (last_request_.timestamp + last_request_.duration_ts_front)) {
      return;
    }

    last_request_ = request;
    requests_.push(request);
    RCLCPP_WARN(rclcpp::get_logger("example"), "Trigger Event!");
  }
}

int TriggerExampleNode::Gtest() {

  auto now = std::chrono::system_clock::now();
  auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  // request gtest
  Config event = config_;
  event.timestamp = milliseconds_since_epoch;
  requests_.push(event);

  RCLCPP_INFO(rclcpp::get_logger("TriggerNode"), "gtest put request example."); 
  return 0;
}




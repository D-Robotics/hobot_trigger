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

#ifndef RECORDER_NODE_H_
#define RECORDER_NODE_H_

#include <cstdlib>
#include <ctime>
#include <stack>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <util.h>

using rclcpp::NodeOptions;
using std::placeholders::_1;

class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~RecorderNode();

  // rosbag缓存数据管理主程序
  int Run();

  // rosbag缓存数据读取
  int Read();

  // rosbag缓存数据清理
  int Clean();
  
  // rosbag数据缓存程序
  int Record();

private:

  std::string cache_path_ = "/home/robot/recorder/";
  long cache_time_ = 60000;
  long cycle_time_ = 60;
  std::string format_ = "mcap";       // mcap or sqlite3
  bool is_all_ = true;
  long mag_bag_size_ = 524288000;
  
  std::stack<std::string> clean_paths_;
  std::vector<std::string> rosbag_paths_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::thread thread_;
};

#endif  // RECORDER_NODE_H_
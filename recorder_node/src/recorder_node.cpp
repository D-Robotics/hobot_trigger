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

#include "include/recorder_node.h"

RecorderNode::RecorderNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {

  // 1. 加载节点配置
  this->declare_parameter<std::string>("cache_path", cache_path_);
  this->declare_parameter<long>("cache_time", cache_time_);
  this->declare_parameter<long>("cycle_time", cycle_time_);
  this->declare_parameter<std::string>("format", format_);
  this->declare_parameter<long>("mag_bag_size", mag_bag_size_);

  this->get_parameter<std::string>("cache_path", cache_path_);
  this->get_parameter<long>("cache_time", cache_time_);
  this->get_parameter<long>("cycle_time", cycle_time_);
  this->get_parameter<std::string>("format", format_);
  this->get_parameter<long>("mag_bag_size", mag_bag_size_);

  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n cache_path: " << cache_path_
       << "\n cache_time(unit: ms): " << cache_time_
       << "\n cycle_time(unit: s): " << cycle_time_
       << "\n format: " << format_
       << "\n mag_bag_size(b): " << mag_bag_size_;
    RCLCPP_WARN(rclcpp::get_logger("RecorderNode"), "%s", ss.str().c_str());
  }

  // 2. 清理过去的缓存数据
  std::string command = "rm -rf " + cache_path_;
  system(command.c_str());

  // 3. 打开rosbag记录线程
  thread_ = std::thread(&RecorderNode::Record, this);

  // 4. 打开rosbag缓存管理定时任务
  timer_ = create_wall_timer(std::chrono::seconds(cycle_time_),
                                std::bind(&RecorderNode::Run, this));

  RCLCPP_INFO(rclcpp::get_logger("RecorderNode"), "RecorderNode start."); 

}

RecorderNode::~RecorderNode() {

  std::system("pkill -2 -f 'ros2 bag record'");  // 发送SIGINT信号给ros2 bag record进程

  if (timer_ != nullptr){
    timer_->cancel();
  }

  // 等待线程结束
  while (thread_.joinable())
  {
    thread_.join();
  }
}

int RecorderNode::Record() {
  std::string command = "ros2 bag record";

  if (is_all_) {
    command += " -a";
  }
  command += " -s " + format_;
  command += " -b " + std::to_string(mag_bag_size_);
  command += " -o " + cache_path_;

  RCLCPP_INFO(rclcpp::get_logger("RecorderNode"), command.c_str()); 
  std::system(command.c_str());
  return 0;
}


int RecorderNode::Run() {
  RCLCPP_INFO(rclcpp::get_logger("RecorderNode"), "Run."); 
  
  rosbag_paths_.clear();
  TraverseDirectory(cache_path_, rosbag_paths_);
  std::sort(rosbag_paths_.begin(), rosbag_paths_.end(), CompareByNumber);

  Read();

  Clean();

  return 0;
} 


int RecorderNode::Read() {

  // 1. 获取当前系统时间，设置清理缓存的截断时间。
  auto now = std::chrono::system_clock::now();
  auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  
  milliseconds_since_epoch -= cache_time_;

  // 2. 生成rosbag子包写入reader实例。
  const rosbag2_cpp::ConverterOptions converter_options({"cdr", "cdr"});

  // 3. 遍历rosbag缓存，将超过截断时间的rosbag数据送入清理队列。
  if (rosbag_paths_.size() <= 1) {
    return 0;
  }

  for(int i = 0; i < rosbag_paths_.size() - 1; i++){

    std::string rosbag_path = rosbag_paths_[i];
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader;
    reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    const rosbag2_storage::StorageOptions storage_options({rosbag_path, format_});
    reader->open(storage_options, converter_options);

    while (reader->has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> message = reader->read_next();
      auto time_stamp = message->time_stamp / 1000000;
      RCLCPP_INFO(rclcpp::get_logger("RecorderNode"), "rosbag_path: %s, time_stamp: %ld", rosbag_path.c_str(), time_stamp); 
      if (time_stamp > milliseconds_since_epoch) {
        reader->close();
        return 0;
      }
      clean_paths_.push(rosbag_path);
      break;
    }
    reader->close();
  }

  return 0;
}


int RecorderNode::Clean() {
  while(!clean_paths_.empty()) {
    std::string command = "rm -rf " + clean_paths_.top();
    RCLCPP_INFO(rclcpp::get_logger("RecorderNode"), command.c_str()); 
    system(command.c_str());
    clean_paths_.pop();
  }
}
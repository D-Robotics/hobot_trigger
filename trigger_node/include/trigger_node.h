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

#ifndef TRIGGER_NODE_H_
#define TRIGGER_NODE_H_

#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>

// #include <iostream>
// #include <string>
// #include <vector>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

using rclcpp::NodeOptions;

struct GPS_POS {
  int latitude = -1;
  int longitude = -1;
};

struct EXTRA_KV {
  std::string key;
  std::string value;
};

struct Config {
  std::string domain;       // Trigger事件domain
  std::string desc;         
  long duration_ts_back;    // 录制trigger 发生后持续时长
  long duration_ts_front;   // 录制tirgger 发生前持续时长
  GPS_POS gps_pos;          // GPS定位
  int level;                // 优先级
  std::string rosbag_path;  // trigger发生后rosbag本地文件路径
  int src_module_id;        // 发生trigger的模块
  int status;               // trigger状态
  std::string strategy_version; // 策略版本号
  long timestamp;           // trigger发生时间戳
  std::vector<std::string> topics;    // 需要记录的话题list，包含话题名和话题类型
  int trigger_type;         // trigger类型
  std::string unique_id;    // 设备唯一标识
  std::string version;      // trigger版本信息
  std::vector<EXTRA_KV> extra_kv;   // 额外信息
};

namespace hobot {
namespace trigger_node {

class TriggerNode : public rclcpp::Node {
 public:

  TriggerNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~TriggerNode();

  // 接受Agentnode话题信息
  void AgentTopicCallback(
    const std_msgs::msg::String::ConstSharedPtr msg);
    
  // 解码json_str，更新到config配置中
  int Decode(std::string& json_str, Config& config);

  // 编码Config中配置信息，生成string
  int Encode(std::string& json_str, Config& result);

  // 加载配置
  int LoadConfig();

  // 保存配置
  int SaveConfig(std::string& path, std::string& str);

  // 记录trigger相关rosbag数据
  int Record();

  // 报告
  void Report();

  // 执行运行流程
  int Run();

 protected:
  // 判断是否启动Trigger模块，并控制订阅器
  virtual int IsStart();

 public:
  Config config_;
  Config last_request_;
  std::queue<Config> requests_;
  std::queue<Config> results_;
  std::string event_msg_sub_topic_name_ = "";

 private:

  std::string cache_path_ = "/home/hobot/recorder/";
  std::string config_file_ = "config/trigger_config.json";
  std::string format_ = "mcap";
  int isRecord_ = 0;
 
  std::string agent_msg_sub_topic_name_ = "/hobot_agent";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      agent_msg_subscription_ = nullptr;

  std::string msg_pub_topic_name_ = "/hobot_trigger";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace trigger_node
}  // namespace hobot

#endif  // TRIGGER_NODE_H_
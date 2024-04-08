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

#include "include/trigger_node.h"

int RecordName2Number(const std::string& str) {
  // 查找"_"的位置
  size_t underscorePos = str.find("_");
  if (underscorePos == std::string::npos) {
    return 0;
  }
  
  // 查找".mcap"的位置
  size_t dotMcapPos = str.find(".mcap");
  if (dotMcapPos == std::string::npos) {
    return 0;
  }
  
  // 提取两个位置之间的子字符串
  std::string extractedString = str.substr(underscorePos + 1, dotMcapPos - underscorePos - 1);
  
  int num = std::stoi(extractedString);

  return num;
}

bool CompareByNumber(const std::string& str1, const std::string& str2) {
  // 提取数字部分并将其转换为整数进行比较

  int num1 = RecordName2Number(str1);
  int num2 = RecordName2Number(str2);

  return num1 < num2;
}

void TraverseDirectory(const std::string& directoryPath, std::vector<std::string>& rosbag_paths) {
  DIR* dir = opendir(directoryPath.c_str());
  if (dir == nullptr) {
    std::cerr << "Failed to open directory: " << directoryPath << std::endl;
    return;
  }

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string fileName = entry->d_name;

    // Skip "." and ".." directories
    if (fileName == "." || fileName == "..")
      continue;

    std::string filePath = directoryPath + "/" + fileName;

    // Check if the entry is a directory
    if (entry->d_type == DT_DIR) {
        // Recursive call for subdirectories
      TraverseDirectory(filePath, rosbag_paths);
    } else {
      std::string format = filePath.substr(filePath.length() - 4, 4);
      if (format != "yaml") {
        rosbag_paths.push_back(filePath);
      }
    }
  }

  closedir(dir);
}


namespace hobot {
namespace trigger_node {

TriggerNode::TriggerNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {

  this->declare_parameter<std::string>("cache_path", cache_path_);
  this->declare_parameter<std::string>("config_file", config_file_);
  this->declare_parameter<std::string>("format", format_);
  this->declare_parameter<int>("isRecord", isRecord_);
  this->declare_parameter<std::string>("agent_msg_sub_topic_name", agent_msg_sub_topic_name_);
  this->declare_parameter<std::string>("event_msg_sub_topic_name", event_msg_sub_topic_name_);
  this->declare_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  this->get_parameter<std::string>("cache_path", cache_path_);
  this->get_parameter<std::string>("config_file", config_file_);
  this->get_parameter<std::string>("format", format_);
  this->get_parameter<int>("isRecord", isRecord_);
  this->get_parameter<std::string>("agent_msg_sub_topic_name", agent_msg_sub_topic_name_);
  this->get_parameter<std::string>("event_msg_sub_topic_name", event_msg_sub_topic_name_);
  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  // 初始化config配置
  int ret = LoadConfig();
  std::string json_str;
  Encode(json_str, config_);

  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n cache_path: " << cache_path_
       << "\n config_file: " << config_file_
       << "\n format: " << format_
       << "\n isRecord(1:record, 0:norecord): " << isRecord_
       << "\n agent_msg_sub_topic_name: " << agent_msg_sub_topic_name_
       << "\n event_msg_sub_topic_name: " << event_msg_sub_topic_name_
       << "\n msg_pub_topic_name: " << msg_pub_topic_name_
       << "\n config detail: " << json_str;
    RCLCPP_WARN(rclcpp::get_logger("hobot_trigger"), "%s", ss.str().c_str());
  }

  agent_msg_subscription_ =
  this->create_subscription<std_msgs::msg::String>(
      agent_msg_sub_topic_name_,
      10,
      std::bind(&TriggerNode::AgentTopicCallback,
                this,
                std::placeholders::_1));

  msg_publisher_ = this->create_publisher<std_msgs::msg::String>(
      msg_pub_topic_name_, 10);

  timer_ = create_wall_timer(std::chrono::seconds(1),
                              std::bind(&TriggerNode::Run, this));

  RCLCPP_WARN(rclcpp::get_logger("hobot_trigger"), "TriggerNode Init Succeed!");
}

TriggerNode::~TriggerNode() {}

int TriggerNode::IsStart() {
  return 0;
}

int TriggerNode::Run() {

  IsStart();

  if (config_.status == 0) {
    return 0;
  }

  if (requests_.empty()) {
    return 0;
  }

  if (isRecord_ == 1) {
    int ret = Record();
    if (ret == -1){
      return 0;
    }
  }

  auto event = requests_.front();
  results_.push(event);
  requests_.pop();

  Report();
  results_.pop();

  RCLCPP_INFO(rclcpp::get_logger("hobot_trigger"), "TriggerNode Run.");
  return 0;
}


int TriggerNode::LoadConfig() {

  if (config_file_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Config file [%s] is empty!",
                 config_file_.data());
    return -1;
  }
  // Parsing config
  std::ifstream ifs(config_file_.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Read config file [%s] fail!",
                 config_file_.data());
    return -1;
  }

  // Init json file
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Parsing config file %s failed",
                 config_file_.data());
    return -1;
  }

  // Load config
  config_.domain = document["domain"].GetString();
  config_.desc = document["desc"].GetString();
  config_.duration_ts_back = document["duration_ts_back"].GetInt();
  config_.duration_ts_front = document["duration_ts_front"].GetInt();
  config_.level = document["level"].GetInt();

  config_.src_module_id = document["src_module_id"].GetInt();
  config_.status = document["status"].GetInt();
  config_.strategy_version = document["strategy_version"].GetString();

  config_.topics.clear();
  for (rapidjson::SizeType i = 0; i < document["topics"].Size(); i++) {
    config_.topics.push_back(document["topics"][i].GetString());
  }

  config_.trigger_type = document["trigger_type"].GetInt();

  config_.unique_id = document["unique_id"].GetString();
  config_.version = document["version"].GetString();

  config_.extra_kv.clear();
  for (rapidjson::SizeType i = 0; i < document["extra_kv"].Size(); i++) {
    EXTRA_KV extra_kv;
    extra_kv.key = document["extra_kv"][i]["key"].GetString();
    extra_kv.value = document["extra_kv"][i]["value"].GetString();
    config_.extra_kv.push_back(extra_kv);
  }

  return 0;
}


int TriggerNode::SaveConfig(std::string& path, std::string& str) {

  // 创建一个 Document 对象并解析 JSON 字符串
  rapidjson::Document document;
  document.Parse(str.c_str());

  // 将 Document 对象序列化为字符串
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);

  // 将字符串写入文件
  std::ofstream file(path);
  if (file.is_open()) {
      file << buffer.GetString();
      file.close();
  } else {
      return -1;
  }

  return 0;
}


int TriggerNode::Record() {

  // 1. 获取recorder_request配置信息
  Config& request = requests_.front();
  long trigger_timestamp = request.timestamp;
  long ts_front = trigger_timestamp - request.duration_ts_front;
  long ts_back = trigger_timestamp + request.duration_ts_back;
  std::string unique_id = request.unique_id;

  // 判断当前时间是否比trigger事件后事件久，否则等待。
  auto now = std::chrono::system_clock::now();
  auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  if (milliseconds_since_epoch < ts_back) {
    return -1;
  }

  int millisec = trigger_timestamp % 1000;
  std::time_t sectimestamp = (trigger_timestamp + 28800000) / 1000; // 转换为秒级时间戳
  std::tm* tm = std::gmtime(&sectimestamp); // 转换为UTC时间
  char buffer[80];
  std::strftime(buffer, 80, "%Y%m%d-%H%M%S", tm); // 将时间格式化为字符串
  std::string time_str(buffer); // 将格式化后的字符串转换为std::string类型
  std::string sub_rosbag_name = unique_id + "_" + time_str + "-" + std::to_string(millisec);
  std::string save_path = "trigger/" + sub_rosbag_name;

  // 2. 生成rosbag子包写入writer实例。
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer;
  const rosbag2_cpp::ConverterOptions converter_options({"cdr", "cdr"});
  writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  writer->open({save_path, format_}, converter_options);

  // 3. 遍历rosbag缓存，读写匹配trigger的rosbag数据
  std::vector<std::string> filename_lists;
  TraverseDirectory(cache_path_, filename_lists);
  std::sort(filename_lists.begin(), filename_lists.end(), CompareByNumber);

  for(auto filename : filename_lists){

    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader;
    const rosbag2_storage::StorageOptions storage_options({filename, format_});
    reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader->open(storage_options, converter_options);

    auto topics = reader->get_all_topics_and_types();
    for (const auto& topic : topics) {
      std::string topic_name = topic.name;
      std::string topic_type = topic.type;

      for(auto request_topic : request.topics){
        if (request_topic == topic_name){   
          writer->create_topic(topic);
        }
      }
    }

    while (reader->has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> message = reader->read_next();
      if ((message->time_stamp / 1000000) < ts_front || (message->time_stamp / 1000000) > ts_back) {
        continue;
      }

      for(auto request_topic : request.topics){
        if (request_topic == message->topic_name){   
          auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
          bag_message->serialized_data = message->serialized_data;
          bag_message->topic_name = message->topic_name;
          bag_message->time_stamp = message->time_stamp;
          writer->write(bag_message);
        }
      }
    }
    reader->close();
  }
  // 4. 将获取的rosbag数据结果，存入结果消息发送队列
  std::string suffix = "";
  if (format_ == "sqlite3") {
    suffix = "db3";
  } else {
    suffix = "mcap";
  }
  std::string rosbag_path = save_path + "/" + sub_rosbag_name + "_0." + suffix;
  request.rosbag_path = rosbag_path;

  // 5. 将trigger配置文件保存下来
  std::string json_str;
  Encode(json_str, request);
  std::string config_path = save_path + "/" + "config.json";
  SaveConfig(config_path, json_str);

  RCLCPP_INFO(rclcpp::get_logger("TriggerNode"), "generate trigger module id: %d, type id: %d, save path: %s", 
    request.src_module_id, request.trigger_type, save_path.c_str());

  // 5. 清理
  writer->close();
  return 0;
}


void TriggerNode::Report() {

  auto result = results_.front();
  std::string json_str;

  Encode(json_str, result);

  // 发布Trigger Event
  std_msgs::msg::String message;
  message.data = json_str;
  msg_publisher_->publish(message);

  std::stringstream ss;
  ss << "Trigger Event Report."
      << " Trigger moudle id: " << result.src_module_id
      << ", type id: " << result.trigger_type
      << "\n Report message: " << json_str;
  RCLCPP_WARN(rclcpp::get_logger("hobot_trigger"), "%s", ss.str().c_str());

}


int TriggerNode::Encode(std::string& json_str, Config& result) {

  rapidjson::Document doc;
  doc.SetObject(); // 创建一个空的Object对象
  rapidjson::Document::AllocatorType& allocator = doc.GetAllocator(); // 获取allocator

  std::string domain = result.domain;
  rapidjson::Value domain_value;
  domain_value.SetString(domain.c_str(), domain.size(), doc.GetAllocator());
  doc.AddMember("domain", domain_value, allocator);

  std::string desc = result.desc;
  rapidjson::Value desc_value;
  desc_value.SetString(desc.c_str(), desc.size(), doc.GetAllocator());
  doc.AddMember("desc", desc_value, allocator);

  doc.AddMember("duration_ts_back", result.duration_ts_back, allocator);
  doc.AddMember("duration_ts_front", result.duration_ts_front, allocator);

  rapidjson::Value gps_value(rapidjson::kObjectType);
  gps_value.AddMember("latitude", result.gps_pos.latitude, allocator);
  gps_value.AddMember("longitude", result.gps_pos.longitude, allocator);
  doc.AddMember("gps_pos", gps_value, allocator);

  doc.AddMember("level", result.level, allocator);

  std::string rosbag_path = result.rosbag_path;
  rapidjson::Value rosbag_path_value;
  rosbag_path_value.SetString(rosbag_path.c_str(), rosbag_path.size(), doc.GetAllocator());
  doc.AddMember("rosbag_path", rosbag_path_value, allocator);

  doc.AddMember("src_module_id", result.src_module_id, allocator);

  std::string strategy_version = result.strategy_version;
  rapidjson::Value strategy_version_value;
  strategy_version_value.SetString(strategy_version.c_str(), strategy_version.size(), doc.GetAllocator());
  doc.AddMember("strategy_version", strategy_version_value, allocator);

  doc.AddMember("timestamp", result.timestamp, allocator);

  rapidjson::Value topics(rapidjson::kArrayType);
  for (auto topic : result.topics) {
    rapidjson::Value element;
    element.SetString(topic.c_str(), topic.size(), doc.GetAllocator());
    topics.PushBack(element, doc.GetAllocator());
  }
  doc.AddMember("topic", topics, doc.GetAllocator());

  doc.AddMember("trigger_type", result.trigger_type, allocator);

  std::string unique_id = result.unique_id;
  rapidjson::Value unique_id_value;
  unique_id_value.SetString(unique_id.c_str(), unique_id.size(), doc.GetAllocator());
  doc.AddMember("unique_id", unique_id_value, allocator);

  std::string version = result.version;
  rapidjson::Value version_value;
  version_value.SetString(version.c_str(), version.size(), doc.GetAllocator());
  doc.AddMember("version", version_value, allocator);

  rapidjson::Value extra_kv_value(rapidjson::kArrayType);
  for (auto extra_kv : result.extra_kv) {
    rapidjson::Value element;

    rapidjson::Value key;
    key.SetString(extra_kv.key.c_str(), extra_kv.key.size(), doc.GetAllocator());
    element.AddMember("key", key, allocator);
    rapidjson::Value value;
    value.SetString(extra_kv.value.c_str(), extra_kv.value.size(), doc.GetAllocator());
    element.AddMember("value", value, allocator);

    extra_kv_value.PushBack(element, doc.GetAllocator());
  }
  doc.AddMember("extra_kv", extra_kv_value, doc.GetAllocator());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  json_str = buffer.GetString();
  return 0;
}


int TriggerNode::Decode(std::string& json_str, Config& config) {
  
  rapidjson::Document document;
  document.Parse(json_str.c_str());

  if (document.HasParseError() || !document.IsObject()) {
    std::cerr << "Invalid JSON format" << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("TriggerNode"), "Invalid JSON format");
    return -1;
  }

  if (!document.HasMember("strategy") || !document["strategy"].IsArray()) {
    RCLCPP_INFO(rclcpp::get_logger("TriggerNode"), "Invalid Trigger Task");
    return -1;
  }

  const rapidjson::Value& strategyArray = document["strategy"];

  if (!strategyArray.Empty()){
    for (rapidjson::SizeType i = 0; i < strategyArray.Size(); ++i) {
      const rapidjson::Value& strategyObject = strategyArray[i];

      if (strategyObject.HasMember("src_module_id")) {
        int src_module_id = strategyObject["src_module_id"].GetInt();
        if(src_module_id != config.src_module_id){
          continue;
        }
        config.src_module_id = src_module_id;

        if (strategyObject.HasMember("duration_ts_back")) {
          config.duration_ts_back = strategyObject["duration_ts_back"].GetInt64();
        }
        if (strategyObject.HasMember("duration_ts_front")) {
          config.duration_ts_front = strategyObject["duration_ts_front"].GetInt64();
        }
        if (strategyObject.HasMember("level")) {
          config.level = strategyObject["level"].GetInt();
        }
        if (strategyObject.HasMember("trigger_type")) {
          config.trigger_type = strategyObject["trigger_type"].GetInt();
        }
        if (strategyObject.HasMember("unique_id")) {
          config.unique_id = strategyObject["unique_id"].GetString();
        }

        if (document.HasMember("version") && document["version"].IsString()) {
          config.version = document["version"].GetString();
        }

        if (document.HasMember("trigger_status") && document["trigger_status"].IsBool()) {
            config.status = document["trigger_status"].GetBool();
        }
      }

    }
  }
  return 0;
}


void TriggerNode::AgentTopicCallback(
    const std_msgs::msg::String::ConstSharedPtr msg) {

  std::string json_str = msg->data;
  Decode(json_str, config_);

  Encode(json_str, config_);
  std::stringstream ss;
  ss << "Updated Trigger Config: " << json_str << "\n";

  RCLCPP_INFO(rclcpp::get_logger("TriggerNode"), "%s", ss.str().c_str());
}

}  // namespace trigger_node
}  // namespace hobot
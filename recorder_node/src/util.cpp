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

#include <util.h>

int RecordName2Number(const std::string& str) {
  // 查找"_"的位置
  size_t underscorePos = str.find("_");
  if (underscorePos == std::string::npos) {
    // std::cout << "Underscore not found" << std::endl;
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

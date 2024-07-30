English| [简体中文](./README_cn.md)

# Function Introduction

The `trigger_node_example` package is developed by TROS based on the custom Trigger basic module, demonstrating the usage of Trigger module. The functionality showcased in this example is subscribing to garbage detection box information and triggering Trigger events based on the number of garbage detection boxes.

This package supports direct subscription to the topic of type `ai_msg/msg/PerceptionTargets`. In the topic callback function, it determines whether to trigger Trigger events, records relevant Rosbag packages for Trigger events, and finally publishes Trigger event topic information to `agent_node`.

# Compilation

## Dependencies

ROS packages:

- ai_msgs
- rclcpp
- rosbag2_cpp
- std_msgs

`ai_msgs` is a custom message format by TROS, used for publishing inference results after algorithm model inference, with the `ai_msgs` package defined in `hobot_msgs`.

`rclcpp` is a C++ client library in ROS2, providing APIs for creating ROS2 nodes, subscribing and publishing topics, invoking services, creating timers, etc.

`rosbag2_cpp` is a library in ROS2 providing C++ interfaces for ROSBAG2 data recording and reading.

`std_msgs` is a standard message package in ROS2 containing common data types like integers, floats, strings, used for communication between ROS nodes.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0 / Linaro GCC 9.3.0

## Compilation

Supports compilation on X3 Ubuntu system and cross-compilation using Docker on PC.

### Compilation Options

1. **BUILD_HBMEM**
   - Zero-Copy Transfer Enable Switch. Enabled by default during Docker cross-compilation (ON), can be disabled during compilation with `-DBUILD_HBMEM=OFF`.
   - On the board-side compilation, the zero-copy transfer method is disabled by default. If zero-copy is required, it can be enabled with `-DBUILD_HBMEM=ON`.
   - When enabled, compilation depends on the `hbm_img_msgs` package and requires using `tros` for compilation.
   - When disabled, compilation and execution do not depend on `hbm_img_msgs` package, supporting compilation using native ROS and `tros`.
   - For zero-copy communication, only subscription to images in the nv12 format is currently supported.

### X3 Ubuntu Board-Side Compilation Version

1. Confirmation of Compilation Environment
   - X3 Ubuntu system is installed on the board.
   - The compilation terminal has set TogetheROS environment variables: `source PATH/setup.bash`. Here, `PATH` represents the installation path of TogetheROS.
   - ROS2 compilation tool colcon has been installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation
   Compilation command: `colcon build --packages-select trigger_node_example --cmake-args -DBUILD_HBMEM=ON`

### Docker Cross-compilation X3 version

1. Compilation environment confirmation
   - Compile in docker, and TogetheROS has been installed in docker. For docker installation, cross-compilation instructions, TogetheROS compilation, and deployment instructions, please refer to the README.md in the robot development platform robot_dev_config repository.

2. Compilation
   - Compilation command:

```shell
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select trigger_node_example \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```


## Notes

# Instructions for use

## Parameters

| Parameter Name          | Type        | Explanation                               | Required | Supported Configurations | Default Value |
| ----------------------  | ----------- | ----------------------------------------- | -------- | ------------------------ | ------------- |
| cache_path              | std::string | Path to the Rosbag folder cached in the runtime environment | No       | Configured according to the actual deployment environment | /home/robot/recorder/ |
| config_file             | std::string | Path to the configuration file for initializing the Trigger module | No | Configured according to the actual deployment environment | config/trigger_config.json |
| format                  | std::string | Format for Trigger to record Rosbag data | No | mcap | mcap |
| isRecord                | int         | Choose whether to record Rosbag data for Trigger events | No | 1: Record / 0: Do not record | 0 |
| agent_msg_sub_topic_name| std::string | Topic name to receive messages from the agent_node node | No | Should be consistent with the agent_node configuration | /hobot_agent |
| event_msg_sub_topic_name| std::string | Topic name to receive Trigger event-related topics | Yes | Configured according to the actual deployment environment |  |
| msg_pub_topic_name      | std::string | Topic name for trigger_node to publish Trigger events | No | Configured according to the actual deployment environment | /hobot_trigger |

## Notes

- The configuration file config_file is in JSON format, with specific configurations as follows:
```json
{
   "domain":"robot",
   "desc":"trigger lane",
   "duration_ts_back":5000,
   "duration_ts_front":5000,
   "level":1,
   "src_module_id": 203,
   "status": 1,
   "strategy_version": "Robot_sweeper_V1.0_20230526",
   "topics": ["/image_raw/compressed", "/ai_msg_mono2d_trash_detection"],
   "trigger_type": 1110,
   "unique_id": "OriginBot002",
   "version":"v1.0.0",
   "extra_kv":[]
}
```
## Run

After successful compilation, copy the generated install path to the Sunrise X3 development board (if compiling on X3, ignore the copy step), and execute the following command to run:


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# The model used in the example in config is copied based on the actual installation path
# If it is a board side compilation (without the -- merge install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., Among them, PKG_NAME is the specific package name.
cp -r install/lib/trigger_node_example/config/ .
cp -r install/lib/mono2d_trash_detection/config/ .

# Start agent node
ros2 run trigger_node_example trigger_node_example --ros-args -p format:=mcap isRecord:=1

```

### **Ubuntu X3 Launch**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# The model used in the example in config is copied based on the actual installation path
# If it is a board side compilation (without the -- merge install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., Among them, PKG_NAME is the specific package name.
cp -r install/lib/trigger_node_example/config/ .
cp -r install/lib/mono2d_trash_detection/config/ .

export CAM_TYPE=mipi

ros2 launch trigger_node_example hobot_trigger_example.launch.py trigger_example_format:=mcap
```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Example models in the config folder, copy according to the actual installation path
cp -r install/lib/trigger_node_example/config/ .
cp -r install/lib/mono2d_trash_detection/config/ .

# Start the agent node detection node
./install/lib/trigger_node_example/trigger_node_example
```

# Result Analysis

## X3 Log Information

```bash
   [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-13-17-31-53-158704-ubuntu-2981490
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [trigger_node_example-1]: process started with pid [2981766]
   [trigger_node_example-1] [WARN] [1683970314.850652382] [hobot_trigger]: Parameter:
   [trigger_node_example-1]  cache_path: /home/robot/recorder/
   [trigger_node_example-1]  config_file: config/trigger_config.json
   [trigger_node_example-1]  format: mcap
   [trigger_node_example-1]  isRecord(1:record, 0:norecord): 1
   [trigger_node_example-1]  agent_msg_sub_topic_name: /hobot_agent
   [trigger_node_example-1]  event_msg_sub_topic_name: /ai_msg_mono2d_trash_detection
   [trigger_node_example-1]  msg_pub_topic_name: /hobot_trigger
   [trigger_node_example-1]  config detail: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"","src_module_id":203,"timestamp":-1,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"v1.0.0\n","version":"v1.0.0\n"}
   [trigger_node_example-1] [WARN] [1683970314.893573769] [hobot_trigger]: TriggerNode Init Succeed!
   [trigger_node_example-1] [WARN] [1683970314.898132256] [example]: TriggerExampleNode Init.
   [trigger_node_example-1] [WARN] [1683970315.931225440] [example]: Trigger Event!
   [trigger_node_example-1] [WARN] [1683970322.178604839] [rosbag2_storage_mcap]: no message indices found, falling back to reading in file order
   [trigger_node_example-1] [WARN] [1683970323.007470033] [hobot_trigger]: Trigger Event Report. Trigger moudle id: 203, type id: 1110
   [trigger_node_example-1]  Report message: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"trigger/OriginBot002_20230513-173155-931/OriginBot002_20230513-173155-931_0.mcap","src_module_id":203,"timestamp":1683970315931,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"OriginBot002","version":"v1.0.0"}
```

# Extended Functionality

## Dispatching Tasks to the Trigger Module

The Trigger module supports the dispatch of Trigger tasks by other nodes to control Trigger configurations. The dispatch is done by publishing on the std_msg topic, sending the task protocol to the Trigger module.

### Trigger Task Protocol
```json
{
   "version": "v0.0.1_20230421",       // Trigger module version information.
   "trigger_status": true,             // Trigger status, '0': off, '1': on.
   "strategy": [
      {
            "src_module_id": 203,      // Module ID where Trigger occurs.
            "trigger_type": 1110,      // Trigger type ID.
            "level": 1,                // Priority of Trigger event.
            "desc": "",                // Description of Trigger module.
            "duration_ts_back": 5000,  // Duration of recording after Trigger occurs.
            "duration_ts_front": 3000  // Duration of recording before Trigger occurs.
      }
   ]
}
```


### Run

On top of starting the Trigger node, in another terminal, publish a topic message with the topic name "/hobot_agent" using std_msgs/String type.
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

ros2 topic pub /hobot_agent std_msgs/String "data: '{\"version\":\"v0.0.1_20230421\",\"trigger_status\":true,\"strategy\":[{\"src_module_id\":203,\"trigger_type\":1110,\"status\":true,\"level\":1,\"desc\":\"test\",\"duration_ts_back\":5000,\"duration_ts_front\":3000}]}'"
```

### Log Information
```shell
   [WARN] [1691670626.026737642] [hobot_trigger]: TriggerNode Init Succeed!
   [WARN] [1691670626.026859316] [example]: TriggerExampleNode Init.
   [INFO] [1691670626.517232775] [TriggerNode]: Updated Trigger Config: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":3000,"gps_pos":{"latitude":-1,"longitude":-1},"level":1,"rosbag_path":"","src_module_id":203,"strategy_version":"Robot_sweeper_V1.0_20230526","timestamp":0,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection","/hobot_visualization"],"trigger_type":1110,"unique_id":"OriginBot002","version":"v0.0.1_20230421","extra_kv":[]}
```

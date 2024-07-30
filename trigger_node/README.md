English| [简体中文](./README_cn.md)

# Function Introduction

**trigger_node package** is a Trigger basic module developed by TROS based on Ros2. It is used to obtain specified Rosbag data after triggering Trigger events. The so-called Trigger is to monitor the changes of messages subscribed by the Trigger module based on the set Trigger mechanism, such as detecting changes in the number of detection frame results, changes in vehicle control information, etc., triggering corresponding Trigger events, and recording Ros2 messages within the specified time interval, thereby helping developers locate and reproduce perception, motion control, and other issues in robot scenes.

This package supports directly subscribing to topics of type `std_msgs/msg/String`, in order to receive Trigger module control information sent by the `agent_node` module. At the same time, it also publishes topics of type `std_msgs/msg/String` to report Trigger events to the `agent_node`, further uploading them to the cloud.

On this basis, the functionality of this package supports developers to use the recorded Trigger information locally without using the D-Robotics robot cloud development platform (Eddie Platform).

# Compilation

## Dependencies

ROS package:

- rclcpp
- rosbag2_cpp
- std_msgs

'rclcpp' is a C++ client library in ROS2, which provides APIs for creating ROS2 nodes, subscribing and publishing topics, calling services, creating timers, and more.

'rosbag2_cpp' is a C++ interface library for ROSBAG2 data recording and reading in ROS2.

'std_msgs' is a standard message package in ROS2, containing common data types such as integers, floats, strings, etc., used for communication between ROS nodes.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0 / Linaro GCC 9.3.0

## Compilation

Supports compilation on X3 Ubuntu system and cross-compilation using Docker on PC.

### Compilation Options

1. BUILD_HBMEM
   - Enables zero-copy transmission. It is enabled by default when cross-compiling with Docker (ON), and can be turned off during compilation by using `-DBUILD_HBMEM=OFF`.
   - By default, zero-copy transmission is disabled during compilation on the board. If zero-copy is required, it can be enabled with `-DBUILD_HBMEM=ON`.
   - When enabled, compilation depends on the 'hbm_img_msgs' package and requires the use of 'tros' for compilation.
   - When disabled, compilation and execution do not depend on the 'hbm_img_msgs' package, supporting compilation using native ROS and 'tros'.
   - For zero-copy communication, only the subscription of nv12 formatted images is currently supported.

### Compilation of Ubuntu Board Version on X3

1. Compilation Environment Confirmation
   - X3 Ubuntu system is installed on the board.
   - The current compilation terminal has set the TogetheROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetheROS.
   - ROS2 compilation tool colcon has been installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation
   Compilation command: `colcon build --packages-select trigger_node --cmake-args -DBUILD_HBMEM=ON`

### Docker Cross-compilation X3 version

1. Compilation environment confirmation
   - Compilation in docker, and TogetheROS has been installed in docker. For docker installation, cross-compilation instructions, TogetheROS compilation and deployment instructions, please refer to the README.md in the robot development platform robot_dev_config repo.

2. Compilation
   - Compilation command:

```shell
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select trigger_node \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

### Compilation of X86 Version on X86 Ubuntu System

1. Compilation environment confirmation
   - X86 Ubuntu version: ubuntu20.04

2. Compilation
   - Compilation command:

   ```shell
   colcon build --packages-select trigger_node  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DBUILD_HBMEM=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Notes# Usage Introduction

## Parameters

| Parameter Name        | Type         | Explanation                                    | Required | Supported Configurations  | Default Value           |
| --------------------- | ------------ | ---------------------------------------------- | -------- | ------------------------- | ------------------------ |
| cache_path            | std::string  | Path to the Rosbag folder cached in the runtime environment | No       | Configured based on actual deployment environment | /home/robot/recorder/ |
| config_file           | std::string  | Path to the initialization configuration file of the Trigger module | No       | Configured based on actual deployment environment | config/trigger_config.json |
| format                | std::string  | Format in which Trigger records Rosbag data | No       | mcap | mcap |
| isRecord              | int          | Choose whether to record Rosbag data for trigger events | No       | 1: record / 0: not record | 0 |
| agent_msg_sub_topic_name | std::string | Topic name to receive from agent_node | No       | Should be consistent with agent_node configuration | /hobot_agent |
| event_msg_sub_topic_name | std::string | Topic name to receive Trigger event related topics | Yes      | Configured based on actual deployment environment |  |
| msg_pub_topic_name    | std::string  | Topic name for trigger_node to publish Trigger events | No       | Configured based on actual deployment environment | /hobot_trigger |

## Notes

- The config_file configuration file is in JSON format, with specific configurations as follows:
```json
  {
   "domain":"robot",                             //Trigger event domain. Triggers of different types, such as vacuum cleaners and humanoid robots, are distinguished by domain to distinguish between different types of robots in different fields
   "desc":"trigger lane",                        //Trigger module description information
   "duration_ts_back":5000,                       //Record the duration of Trigger occurrence
   "duration_ts_front":5000,                     //Record the duration before the occurrence of Tiger
   "level":1,                                    //The priority of Trigger events. When multiple different Triggers occur, a total node can be used to filter out some high or low priority Trigger events
   "src_module_id": 203,                          //The module ID where the Trigger occurred, used to manage different Trigger modules and meet the management needs of different Trigger modules in the business
   "status": 1,                                    //Trigger status, '0': closed, '1': open
   "strategy_version": "Robot_sweeper_V1.0_20230526",   //The version number of the Trigger module policy
   "topics": ["/image_raw/compressed", "/ai_msg_mono2d_trash_detection"],   //A list of topics to be recorded, including topic names
   "trigger_type": 1110,                            //Trigger type ID. Each Trigger module does not have only one triggering situation, for example, detecting 2 garbage triggers is of the same type, and detecting 3 garbage triggers is of the same type
   "unique_id": "OriginBot002",                     //Device Unique Identification
   "version":"v1.0.0",                              //Trigger module version information
   "extra_kv":[]                                    //Trigger module version and other redundant extension information can be recorded here
   }
```

## Running

After successful compilation, the corresponding `libtrigger_node.so` shared library will be generated in the install path as the base class of the trigger module. `trigger_node` cannot be directly run and used. In this code repository, we provide `trigger_node_example` as an example for using the trigger module. Please go to the corresponding repository path for usage experience.

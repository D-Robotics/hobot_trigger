English| [简体中文](./README_cn.md)

# Function Introduction

The `recorder_node` package is a Ros2-based Rosbag data caching management package developed by Horizon Robotics. Its main function is to record topic data and provide Rosbag cached data for the `recorder_node` module. By managing the cache, unnecessary Rosbag data can be cleaned up through scheduled tasks to ensure that there is sufficient space in the operating environment resources, and it is also convenient for the Trigger module to quickly capture Rosbag data within a specified time interval.

# Compilation

## Dependencies

ROS packages:

- rclcpp
- rosbag2_cpp

`rclcpp` is a C++ client library in ROS2 that provides APIs for creating ROS2 nodes, subscribing and publishing topics, calling services, creating timers, and more.

`rosbag2_cpp` is a library in ROS2 that provides a C++ interface for recording and reading ROSBAG2 data.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Supports compiling on X3 Ubuntu system and cross-compiling with Docker on PC.

### Compilation Options

1. BUILD_HBMEM
   - Switch to enable zero-copy transmission. It is enabled by default when cross-compiling with Docker (ON) and can be turned off during compilation with `-DBUILD_HBMEM=OFF`.
   - When compiling on the board, the zero-copy transmission switch is off by default. If zero-copy is needed, it can be enabled with `-DBUILD_HBMEM=ON`.
   - If enabled, the compilation will depend on the `hbm_img_msgs` package and tros will be needed for compilation.
   - If disabled, compilation and execution will not depend on the `hbm_img_msgs` package, supporting compilation with native ROS and tros.
   - Currently, zero-copy communication only supports subscribing to images in nv12 format.

### Compilation for X3 Ubuntu Board

1. Compilation Environment Setup
   - Ensure that the X3 Ubuntu system is installed on the board.
   - The compilation terminal has set the TogetheROS environment variable: `source PATH/setup.bash`. Replace PATH with the installation path of TogetheROS.
   - Install ROS2 compilation tool colcon with command: `pip install -U colcon-common-extensions`

2. Compilation
   Compilation command: `colcon build --packages-select recorder_node --cmake-args -DBUILD_HBMEM=ON`

### Docker Cross-Compilation for X3 Version1. Compilation Environment Confirmation

   - Compile in docker, with TogetheROS installed in the docker environment. For instructions on docker installation, cross-compilation, TogetheROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.

2. Compilation

   - Compilation command:

```shell
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select recorder_node \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

### Compiling on X86 Ubuntu System for X86 Version

1. Compilation Environment Confirmation

   - X86 Ubuntu Version: Ubuntu 20.04

2. Compilation

   - Compilation command:

   ```shell
   colcon build --packages-select recorder_node  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DBUILD_HBMEM=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Notes

# User Guide

## Parameters

| Parameter Name | Type         | Description                                 | Required | Supported Configurations | Default Value               |
| -------------- | ------------ | ------------------------------------------- | -------- | ------------------------ | --------------------------- |
| cache_path     | std::string  | Path to the rosbag folder cached in the runtime environment | No       | Configured based on actual deployment environment | /home/hobot/recorder/      |
| cache_time     | long         | Time to retain cached data (unit: ms)       | No       | Configured based on actual deployment environment | 60000                        || cycle_time | long | Cycle time for clearing cache data (unit: s) | No | Configured based on actual deployment environment | 60 |
| format | std::string | Format of recording rosbag data | No | mcap | mcap |
| mag_bag_size | long | Size of each cached data package in rosbag (unit: bit) | No | Configured based on actual deployment environment | 524288000 |

## Running

After successful compilation, copy the generated installation path to the Horizon X3 development board (ignore the copying step if compiling on X3), and run the following commands:


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Start the recorder node
ros2 run recorder_node recorder_node

```

### **Ubuntu X3 Launch Start**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Start the recorder node
ros2 launch recorder_node hobot_recorder.launch.py
```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Start the recorder node detection node
./install/lib/recorder_node/recorder_node
```

# Results Analysis

## X3 Log Information

```bash
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-31-18-30-18-237313-ubuntu-521540
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [recorder_node-1]: process started with pid [521816]
[recorder_node-1] [WARN] [1685529019.332386609] [RecorderNode]: This is hobot recorder node!
[recorder_node-1] [WARN] [1685529019.449256848] [RecorderNode]: Parameter:[recorder_node-1]  cache_path: /home/hobot/recorder/
[recorder_node-1]  cache_time(unit: ms): 60000
[recorder_node-1]  cycle_time(unit: s): 60
[recorder_node-1]  format: mcap
[recorder_node-1]  mag_bag_size(b): 524288000
[recorder_node-1] [INFO] [1685529023.407010351] [rosbag2_transport]: Listening for topics...
[recorder_node-1] [INFO] [1685529023.444270756] [rosbag2_transport]: Subscribed to topic '/rosout'
[recorder_node-1] [INFO] [1685529023.474319379] [rosbag2_transport]: Subscribed to topic '/parameter_events'
[recorder_node-1] [INFO] [1685529023.497127930] [rosbag2_transport]: Subscribed to topic '/image_raw'
[recorder_node-1] [INFO] [1685529023.502961220] [rosbag2_transport]: Subscribed to topic '/image'
[recorder_node-1] [INFO] [1685529023.522887125] [rosbag2_transport]: Subscribed to topic '/hbmem_img0x22011208050701201313080305190b1b'
[recorder_node-1] [INFO] [1685529023.542508750] [rosbag2_transport]: Subscribed to topic '/camera_info'
[recorder_node-1] [INFO] [1685529023.613529769] [rosbag2_transport]: Subscribed to topic '/ai_msg_mono2d_trash_detection'
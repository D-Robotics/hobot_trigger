# 功能介绍

recorder_node package 是地平线基于Ros2开发的rosbag数据缓存管理功能包。主要功能，是记录话题数据，为 recorder_node 模块提供rosbag缓存数据。同时通过缓存管理方式，通过定时任务清理不再需要保存的rosbag数据，保证运行环境资源空间充足，同时便于trigger模块快速截取指定时间区间rosbag数据。

# 编译

## 依赖库

ros package：

- rclcpp
- rosbag2_cpp

'rclcpp' 是 ROS2 中的一个C++客户端库，提供了用于创建 ROS2 节点、订阅和发布话题、调用服务、创建定时器等功能的API。

'rosbag2_cpp' 是 ROS2 中的一个ROSBAG2数据记录和读取 C++ 接口的库。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### 编译选项

1. BUILD_HBMEM
   - 零拷贝传输方式使能开关。Docker交叉编译时默认打开(ON), 编译时可以通过-DBUILD_HBMEM=OFF关闭。
   - 在板端编译时，零拷贝传输方式使能开关默认是关闭的。如果需要依赖零拷贝，可以通过-DBUILD_HBMEM=ON打开。
   - 如果打开，编译会依赖hbm_img_msgs package，并且需要使用tros进行编译。
   - 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
   - 对于零拷贝通信方式，当前只支持订阅nv12格式图片。

### Ubuntu板端编译X3版本

1. 编译环境确认
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetheROS环境变量：`source PATH/setup.bash`。其中PATH为TogetheROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`

2. 编译
 编译命令：`colcon build --packages-select recorder_node --cmake-args -DBUILD_HBMEM=ON`


### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetheROS。docker安装、交叉编译说明、TogetheROS编译和部署说明详见机器人开发平台 robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

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

### X86 Ubuntu系统上编译 X86版本

1. 编译环境确认

   - x86 ubuntu版本: ubuntu20.04

2. 编译

   - 编译命令：

   ```shell
   colcon build --packages-select recorder_node  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DBUILD_HBMEM=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## 注意事项

# 使用介绍

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| cache_path  | std::string | 缓存在运行环境中的rosbag文件夹路径 | 否 | 根据实际部署环境配置 | /home/hobot/recorder/ |
| cache_time | long | 缓存数据保留的时间(单位:ms) | 否 | 根据实际部署环境配置 | 60000 |
| cycle_time | long | 清理缓存数据循环的时间(单位:s) | 否 | 根据实际部署环境配置 | 60000 |
| format | std::string | 记录rosbag数据的格式 | 否 | mcap | mcap/sqlite3 |
| mag_bag_size | long | rosbag每个缓存数据包大小(单位:bit) | 否 | 根据实际部署环境配置 | 524288000 |

## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动recorder node
ros2 run recorder_node recorder_node

```

### **Ubuntu X3 Launch启动**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动recorder node
ros2 launch recorder_node hobot_recorder.launch.py
```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# 启动recorder node检测node
./install/lib/recorder_node/recorder_node
```

# 结果分析

## X3 日志信息

```bash
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-31-18-30-18-237313-ubuntu-521540
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [recorder_node-1]: process started with pid [521816]
[recorder_node-1] [WARN] [1685529019.332386609] [RecorderNode]: This is hobot recorder node!
[recorder_node-1] [WARN] [1685529019.449256848] [RecorderNode]: Parameter:
[recorder_node-1]  cache_path: /home/hobot/recorder/
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
```
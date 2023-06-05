# 功能介绍

trigger_node_example package 是地平线在自定义trigger基础模块基础上，开发的Trigger模块使用示例。本示例展示的功能，是订阅垃圾检测框信息，根据垃圾检测框的数量，判断是否触发trigger事件的示例。

本package支持直接订阅ai_msg/msg/PerceptionTargets类型的话题，在话题回调函数中，判断是否触发trigger事件，并记录trigger事件相关的rosbag包，最后向agent_node发布trigger事件话题信息。

# 编译

## 依赖库

ros package：

- ai_msgs
- rclcpp
- rosbag2_cpp
- std_msgs

'ai_msgs' 是地平线自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs package定义在hobot_msgs中。

'rclcpp' 是 ROS2 中的一个C++客户端库，提供了用于创建 ROS2 节点、订阅和发布话题、调用服务、创建定时器等功能的API。

'rosbag2_cpp' 是 ROS2 中的一个ROSBAG2数据记录和读取 C++ 接口的库。

'std_msgs' 是 ROS2 中的一个标准消息包，包含了常用的数据类型，如整数、浮点数、字符串等，用于ROS节点之间的通信。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3
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
 编译命令：`colcon build --packages-select trigger_node_example --cmake-args -DBUILD_HBMEM=ON`


### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetheROS。docker安装、交叉编译说明、TogetheROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

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


## 注意事项

# 使用介绍

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| cache_path  | std::string | 缓存在sd卡中的rosbag文件夹路径 | 否      | 根据实际部署环境配置 | /home/hobot/recorder/ |
| config_file | std::string | trigger模块初始化配置文件路径 | 否 | 根据实际部署环境配置 | config/trigger_config.json |
| format | std::string | trigger记录rosbag数据的格式 | 否 | mcap | mcap |
| isRecord | int | 选择trigger事件是否记录rosbag数据 | 否 | 1:记录 / 0:不记录 | 0 |
| agent_msg_sub_topic_name  | std::string | 接收agent_node节点的的topic名 | 否      | 需要与agent_node配置一致 | /hobot_agent |
| event_msg_sub_topic_name  | std::string | 接收trigger事件相关话题的topic名 | 是      | 需要与agent_node配置一致 |  |
| msg_pub_topic_name  | std::string | trigger_node发布trigger事件的话题名 | 否      | 根据实际部署环境配置 | /hobot_trigger |

## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/trigger_node_example/config/ .

# 启动agent node
ros2 run trigger_node_example trigger_node_example

```

### **Ubuntu X3 Launch启动**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/trigger_node_example/config/ .
cp -r install/lib/mono2d_trash_detection/config/ .

export CAM_TYPE=mipi

ros2 launch trigger_node_example hobot_trigger_example.launch.py

```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/trigger_node_example/config/ .

# 启动agent node检测node
./install/lib/trigger_node_example/trigger_node_example
```

# 结果分析

## X3 日志信息

```bash
   [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-13-17-31-53-158704-ubuntu-2981490
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [trigger_node_example-1]: process started with pid [2981766]
   [trigger_node_example-1] [WARN] [1683970314.850652382] [hobot_trigger]: Parameter:
   [trigger_node_example-1]  cache_path: /home/hobot/recorder/
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
   [trigger_node_example-1]  Report message: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"trigger/OriginBot002_20230513-173155-931/OriginBot002_20230513-173155-931_0.mcap","src_module_id":203,"timestamp":1683970315931,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"v1.0.0","version":"v1.0.0"}
```
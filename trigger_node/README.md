# 功能介绍

trigger_node package 是地平线基于Ros2开发的trigger基础模块，用于在触发Trigger事件后，获取指定rosbag数据的功能包。所谓trigger，是在设定好已有trigger机制基础上，监测trigger模块订阅的消息变化，例如检测框结果数量变化，小车控制信息变化等，触发对应trigger事件，记录指定时间区间内的ros2消息，从而帮助开发人员定位和复现机器人场景中的感知、规控等问题。

本package支持直接订阅std_msg/msg/String类型的话题，用以接受agent_node模块发出的trigger模块控制信息，同时也通过发布std_msg/msg/String类型的话题，将trigger事件上报给agent_node，进一步上传到云端。

在此基础上，本package功能支持开发者在不使用地平线机器人云开发平台（艾迪平台）基础上使用，所记录的trigger信息将保存在本地。


# 编译

## 依赖库

ros package：

- rclcpp
- rosbag2_cpp
- std_msgs

'rclcpp' 是 ROS2 中的一个C++客户端库，提供了用于创建 ROS2 节点、订阅和发布话题、调用服务、创建定时器等功能的API。

'rosbag2_cpp' 是 ROS2 中的一个ROSBAG2数据记录和读取 C++ 接口的库。

'std_msgs' 是 ROS2 中的一个标准消息包，包含了常用的数据类型，如整数、浮点数、字符串等，用于ROS节点之间的通信。

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
 编译命令：`colcon build --packages-select trigger_node --cmake-args -DBUILD_HBMEM=ON`


### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetheROS。docker安装、交叉编译说明、TogetheROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

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

### X86 Ubuntu系统上编译 X86版本

1. 编译环境确认

   - x86 ubuntu版本: ubuntu20.04

2. 编译

   - 编译命令：

   ```shell
   colcon build --packages-select trigger_node  \
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
| cache_path  | std::string | 缓存在运行环境中的rosbag文件夹路径 | 否      | 根据实际部署环境配置 | /home/hobot/recorder/ |
| config_file | std::string | trigger模块初始化配置文件路径 | 否 | 根据实际部署环境配置 | config/trigger_config.json |
| format | std::string | trigger记录rosbag数据的格式 | 否 | mcap | mcap |
| isRecord | int | 选择trigger事件是否记录rosbag数据 | 否 | 1:记录 / 0:不记录 | 0 |
| agent_msg_sub_topic_name  | std::string | 接收agent_node节点的的topic名 | 否      | 需要与agent_node配置一致 | /hobot_agent |
| event_msg_sub_topic_name  | std::string | 接收trigger事件相关话题的topic名 | 是      | 需要与agent_node配置一致 |  |
| msg_pub_topic_name  | std::string | trigger_node发布trigger事件的话题名 | 否      | 根据实际部署环境配置 | /hobot_trigger |

## 注意事项

- config_file配置文件格式为json格式，具体配置如下：
  {
   "domain":"robot",          // Trigger事件domain
   "desc":"trigger lane",     // trigger描述信息
   "duration_ts_back":5000,   // 录制trigger发生后持续时长
   "duration_ts_front":5000,  // 录制tirgger 发生前持续时长
   "level":1,                 //优先级
   "src_module_id": 203,      // 发生trigger的模块
   "status": 1,               // trigger状态
   "strategy_version": "Robot_sweeper_V1.0_20230526",   //trigger策略版本
   "topics": ["/image_raw/compressed", "/ai_msg_mono2d_trash_detection"],  // 需要记录的话题list，包含话题名
   "trigger_type": 1110,      // trigger类型
   "unique_id": "OriginBot002",  // 设备唯一标识
   "version":"v1.0.0",        // trigger module 版本
   "extra_kv":[]              // 冗余扩展信息
  }

## 运行

编译成功后，将在install路径下生成对应 libtrigger_node.so 动态链接库，作为trigger模块的基类，trigger_node 并不可以直接运行使用，在本代码仓库中，我们提供了 trigger_node_example 作为trigger模块使用的示例，请移步到对应仓库路径下使用体验。
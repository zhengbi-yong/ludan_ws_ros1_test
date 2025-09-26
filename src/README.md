## bipedal-robot

该程序是没有手臂版本。

该程序跑在ubuntu20.04上，使用ROS1-Noetic，需要x86架构电脑，**比较吃cpu**。

该程序使用OCS2和ros-control框架，控制方法是非线性模型预测控制，模型采用质心动力学。

该程序是基于[livelybot_dynamic_control](https://github.com/HighTorque-Robotics/livelybot_dynamic_control)、[hunter_bipedal_control](https://bridgedp.github.io/hunter_bipedal_control)以及[legged_control](https://github.com/qiayuanl/legged_control)上进行一些改进，主要是对**约束**进行了修改。

## Ludan 结构化命名与配置

为了让“ludan”机器人在扩展不同的肢体时保持清晰的命名，本仓库新增了结构化的电机配置方式。所有控制节点都会读取统一的参数，并按照 `机器人名_肢体名_关节名` 的格式在 `/joint_states` 中发布电机名称。

## 仓库结构概览

整个工作空间采用标准的 `catkin` 结构，`src/` 目录下的主要功能包如下：

- `dmbot_serial`：统一的串口桥接节点，负责读取参数服务器中的电机布局（`motor_layout`）并与 STM32 电机驱动板通信，同时在 `/joint_states` 发布反馈信息。
- `dmbot_serial_leftarm`、`dmbot_serial_neck` 等：在旧版本中用于固定布局的串口节点，现在可以直接由 `dmbot_serial` 统一替代。
- `simple_hybrid_joint_controller*`：针对不同肢体的 ros-control 控制器及其 `launch` 文件，常用于真实机械臂/腿的初始化与调试。
- `legged_common`、`legged_hw`、`legged_examples`：双足机器人控制的核心算法、硬件接口与示例配置，对应 OCS2 + ros-control 框架。
- `right_arm_hw`、`right_arm_moveit_config`、`wanren_arm`：右臂硬件接口与 MoveIt! 规划配置，用于上肢控制与轨迹规划。
- `test_led`、`motor_control_gui4a.py` 等工具：提供基础的硬件连通性测试和 GUI 调试脚本。

建议先阅读各包下的 `README` 或 `launch` 文件，了解节点命名与话题接口，再按照需求组合使用。

### 关键参数

- `~robot_name`：机器人名称，默认值为 `ludan`。
- `~motor_layout`：电机布局列表。每个元素需要包含 `limb`（肢体名称）、`joint`（关节名称）以及 `type`（电机型号）。支持的肢体名称不限于 `left_arm`、`right_arm`、`left_leg`、`right_leg`、`waist`、`neck`，也可以自定义，例如 `arm_3`、`aux_leg_1` 等。
- `~num_motors`：可选备用参数。当未提供 `motor_layout` 时用于指定电机数量。

### 示例配置

可以使用 `rosparam` 或 YAML 文件为机器人配置布局，例如：

```yaml
robot_name: ludan
motor_layout:
  - {limb: right_arm, joint: shoulder_yaw,  type: 10010l}
  - {limb: right_arm, joint: shoulder_pitch, type: 10010l}
  - {limb: right_arm, joint: elbow,         type: 6248p}
  - {limb: right_arm, joint: wrist_yaw,     type: 4340}
  - {limb: left_arm,  joint: shoulder_yaw,  type: 6248p}
  - {limb: left_arm,  joint: wrist_roll,    type: 4340}
  - {limb: waist,     joint: yaw,           type: 10010l}
  - {limb: neck,      joint: pitch,         type: 4340}
```

如果未来需要为 ludan 安装 6 个机械臂，只需在 `motor_layout` 中继续添加新的肢体和关节即可，控制程序会自动识别数量并生成对应的串口通信帧。

### 多末端控制板的推荐配置流程

当机器人接入多个串口控制板（例如 `/dev/mcu_neck`、`/dev/mcu_leftarm`、`/dev/mcu_rightarm`、`/dev/mcu_leftleg`、`/dev/mcu_rightleg`、`/dev/mcu_waist`）时，可以为每块控制板单独启动一个 `dmbot_serial::robot` 节点，并通过命名空间隔离参数：

1. 为每个控制板准备一份 YAML 布局文件，示例：

   ```yaml
   # config/neck.yaml
   robot_name: ludan
   motor_layout:
     - {limb: neck, joint: pitch, type: 4340}
     - {limb: neck, joint: yaw,   type: 4340}
   ```

   ```yaml
   # config/right_leg.yaml
   robot_name: ludan
   motor_layout:
     - {limb: right_leg, joint: hip_yaw,   type: 10010l}
     - {limb: right_leg, joint: hip_roll,  type: 10010l}
     - {limb: right_leg, joint: hip_pitch, type: 10010l}
     - {limb: right_leg, joint: knee,      type: 6248p}
     - {limb: right_leg, joint: ankle,     type: 4340}
   ```

2. 在运行前，依次加载参数并指定串口端口，例如：

   ```bash
   rosparam load config/neck.yaml /neck_bridge
   roslaunch dmbot_serial test_motor.launch \
     port:=/dev/mcu_neck robot_name:=ludan __ns:=neck_bridge
   ```

   ```bash
   rosparam load config/right_leg.yaml /right_leg_bridge
   roslaunch dmbot_serial test_motor.launch \
     port:=/dev/mcu_rightleg robot_name:=ludan __ns:=right_leg_bridge
   ```

   这样每个命名空间都会生成独立的 `/joint_states` 子话题（例如 `/neck_bridge/joint_states`），同时保留统一的命名规则，便于在上层控制器中聚合。

3. 若需要集中使用所有电机的关节状态，可以在上层节点中对多个命名空间的 `joint_states` 进行 remap 或编写聚合节点。

`dmbot_serial` 会自动按照 `motor_layout` 的长度调整串口帧，因此只需保证 YAML 文件与实际接线顺序一致即可，无需修改 C++ 源码。

### 使用建议

1. 肢体名称会自动转为小写并替换为空格的字符，因此建议直接使用英文或数字组合，例如 `left_arm`、`leg_front_left`、`arm_extra_1`。
2. 若某些肢体暂时没有安装，可保留空列表或直接不在 `motor_layout` 中声明，代码会保持兼容。
3. 所有串口桥接脚本、测试程序都会根据 `motor_layout` 自动调整数组长度，无需手工修改常量。

## 学习
1. 理论框架

该程序的数学原理框架可看[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

2. 程序框架

该程序框架是采用ros-control标准格式，对ros-control不太了解的兄弟可先参考用ros-control控制达妙电机的例程[dm-control](https://gitee.com/xauter/dm-control)

## 安装
***安装需要ros基础，这里使用catkin build编译，而不是catkin_make***
### 安装依赖

- [OCS2](https://leggedrobotics.github.io/ocs2/installation.html#prerequisites)

- [ROS1-Noetic](http://wiki.ros.org/noetic)

### 安装 ROS1-Noetic
 这个网上安装教程很多，很简单。
 
### 安装 OCS2
***注意：安装OCS2要保证网络好***

- C++ compiler with C++11 support

- Eigen (v3.3)
```shell
sudo apt-get update
sudo apt-get install libeigen3-dev
```

- Boost C++ (v1.71)
```shell
wget https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2
tar --bzip2 -xf boost_1_71_0.tar.bz2
cd boost_1_71_0
./bootstrap.sh
sudo ./b2 install
```
如果解压boost_1_71\_0.tar.bz2失败，可以自己去官网下载boost_1_71_0.tar.bz2,官网链接如下：

[https://www.boost.org/users/history/](https://www.boost.org/users/history/)


- 安装剩余依赖
```shell
sudo apt install ros-noetic-catkin
sudo apt install libglpk-dev libglfw3-dev
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-grid-map-msgs
sudo apt install ros-noetic-octomap-msgs
sudo apt install libreadline-dev
sudo apt install libcgal-dev
sudo apt update && sudo apt install checkinstall
sudo apt-get install libserialport0 libserialport-dev
sudo apt install ros-noetic-serial
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-joy-teleop
```

- 安装相关库
1. 针对ocs2单独创建一个工作空间
```shell
mkdir -p ~/ocs2_ws/src
cd ~/ocs2_ws/src
```
2. 下载功能包***（先不编译）***

在~/ocs2_ws/src目录下，打开终端，输入：
```shell
 git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
 git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
 git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
 git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01
 git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git
 git clone https://github.com/ANYbotics/grid_map.git
 git clone https://github.com/leggedrobotics/ocs2.git
```
![](./src/docs/catalog_folder.png)

3. 编译Raisim
```shell
cd ~/ocs2_ws/src/raisimLib
mkdir build
cd build
cmake .. 
make -j4 && sudo checkinstall
```

4. ONNX Runtime 依赖
```shell
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
tar xf onnxruntime-linux-x64-1.7.0.tgz
mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime
mkdir -p ~/.local/share/onnxruntime/cmake/
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/onnxruntime/cmake/
```
然后需要在.bashrc手动设置环境变量，否则在后续安装ocs2时，容易找不到该软件包
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在最后一行输入：
```shell
export onnxruntime_DIR=~/.local/
export LD_LIBRARY_PATH=~/.local/lib:${LD_LIBRARY_PATH}
```

5. 编译（最关键的步骤来了，**编译需要在ocs2_ws目录下进行**）

首先，优先编译elevation_mapping_cupy

***注意：一定要添加catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo，如下所示***
```shell
cd ~/ocs2_ws/
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build elevation_mapping_cupy
```
没报错的话，编译grid_map
```shell
catkin build grid_map
```
没报错的话，编译hpp-fcl
```shell
catkin build hpp-fcl
```
没报错的话，编译pinocchio
```shell
catkin build pinocchio
```
没报错的话，编译ocs2_robotic_assets
```shell
catkin build ocs2_robotic_assets
```
没报错的话，编译ocs2（这一步编译要等很长时间）
```shell
catkin build ocs2
```

6. 测试

编译完成之后需要在.bashrc手动设置环境变量
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在.bashrc文件里最后一行输入：
```shell
source ~/ocs2_ws/devel/setup.bash
```
然后在终端输入：
```shell
source .bashrc
```
然后启动ocs2的一个四足例程（首次运行需要等一下，ocs2有个预先计算，等待rviz出现图形）
```shell
roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
```
<img src="./src/docs/leg_example.png" width="500" height="auto">

OCS2安装成功！！！！

### 安装和编译双足控制程序

首先打开终端，输入：
```shell
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```
然后把gitee上的src文件夹放到catkin_ws目录下，如下所示（不能有其他东西）
![](./src/docs/src.png)

打开终端，输入：
```shell
cd ~/catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```
然后在.bashrc文件里最后一行输入：
```
source ~/catkin_ws/devel/setup.bash
```
然后打开终端，输入：
```
cd
source .bashrc
```

## 仿真与实机的运行
***注意：一开始运行实机前最好先运行仿真，检查仿真没问题后，再运行实机***
### 仿真，分两种情况，一种是有遥控器，一种是没有遥控器
#### 没有遥控器
1. 运行gazebo仿真并且载入控制器:

```shell
roslaunch legged_controllers one_start_gazebo.launch
```
<img src="./src/docs/sim.png" width="600" height="auto">

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

3. 新建一个终端，发布/reset_estimation话题，重置状态:
```shell
rostopic pub --once /reset_estimation std_msgs/Float32 "data: 0.0" 
```
4. 新建一个终端，发布/cmd_vel话题，**这里速度不能为0**，不然话题无法发布出去:
```bash
rosrun rqt_robot_steering rqt_robot_steering 
```
<img src="./src/docs/vel.png" width="300" height="auto">

5. 新建一个终端，开启控制器：
```shell
rostopic pub --once /load_controller std_msgs/Float32 "data: 0.0" 
```
6. 最后发布/set_walk话题:
```shell
rostopic pub --once /set_walk std_msgs/Float32 "data: 0.0" 
```


rostopic pub --once /controllers/legged_controller/cmd_joint_velocity std_msgs/Float64MultiArray '{
  "layout": {
    "dim": [
      {"label": "joints", "size": 12, "stride": 12}
    ],
    "data_offset": 0
  },
  "data": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
}'

#### 有遥控器（能支持ros的遥控器都能直接用）

1. 运行gazebo仿真并且载入控制器:
```shell
roslaunch legged_controllers one_start_gazebo.launch
```
2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

后面的步骤就使用遥控器按键，每一个按键对应一个话题，对应关系如下图所示：

<img src="./src/docs/handle.png" width="500" height="auto">

### 实机

1. 给stm32h7开发板烧录下位机程序，上电，此时所有电机应该全亮绿灯

2. 在你的电脑里运行上位机程序
首先检查开发板和陀螺仪的连接
```shell
cd
ls /dev/ttyACM*
```
<img src="./src/docs/dev2.png" width="450" height="auto">

***注意：/dev/ttyACM0是单片机设备，/dev/ttyACM1是IMU设备，两者要各自对应***

然后给用usb连接的开发板和陀螺仪设置权限
```shell
sudo chmod -R 777 /dev/ttyACM*
```
接着运行上位机程序
```shell
roslaunch legged_controllers one_start_real.launch
```
此时rviz中的机器人模型不动，并且姿势和实机一样

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在现实世界里扶正机器人，机器人此时稳定站立。

后面的步骤和上面仿真一样，另外遥控器连接后打开遥控器开关，它会自动发布cmd\_cel话题，不需要再运行rosrun rqt\_robot\_steering rqt_robot\_steering 




## 参考

### 代码参考

https://github.com/bridgedp/hunter_bipedal_control

https://github.com/HighTorque-Robotics/livelybot_dynamic_control

https://github.com/qiayuanl/legged_control

### 文献参考
[OCS2安装参考1](https://blog.csdn.net/Study__ing_/article/details/140177463?fromshare=blogdetail&sharetype=blogdetail&sharerId=140177463&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2安装参考2](https://blog.csdn.net/m0_52545777/article/details/140276558?fromshare=blogdetail&sharetype=blogdetail&sharerId=140276558&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2代码解析：Quadrotor]https://zhuanlan.zhihu.com/p/687952253

```
# State Estimation

[1] Flayols T, Del Prete A, Wensing P, et al. Experimental evaluation of simple estimators for humanoid robots[C]//2017 IEEE-RAS 17th International Conference on Humanoid Robotics (Humanoids). IEEE, 2017: 889-895.

[2] Bloesch M, Hutter M, Hoepflinger M A, et al. State estimation for legged robots-consistent fusion of leg kinematics and IMU[J]. Robotics, 2013, 17: 17-24.

# MPC

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] Grandia R, Jenelten F, Yang S, et al. Perceptive Locomotion Through Nonlinear Model-Predictive Control[J]. IEEE Transactions on Robotics, 2023.

[5] Sleiman J P, Farshidian F, Minniti M V, et al. A unified mpc framework for whole-body dynamic locomotion and manipulation[J]. IEEE Robotics and Automation Letters, 2021, 6(3): 4688-4695.

# WBC

[6] Bellicoso C D, Gehring C, Hwangbo J, et al. Perception-less terrain adaptation through whole body control and hierarchical optimization[C]//2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids). IEEE, 2016: 558-564.

[7] Kim D, Di Carlo J, Katz B, et al. Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control[J]. arXiv preprint arXiv:1909.06586, 2019.
```

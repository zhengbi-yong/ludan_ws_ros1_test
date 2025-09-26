# Ludan 双足机器人控制工作空间

## 项目概览
- **系统框架**：基于 ROS 1 Noetic、ros-control 与 OCS2，融合质心动力学建模与非线性模型预测控制（NMPC）。
- **硬件接口**：统一的串口桥接节点 `dmbot_serial`，支持带有最多 30 个电机的多肢体配置，并对旧的固定布局节点保持兼容。
- **功能包划分**：`legged_*` 系列负责硬件抽象层、控制算法与示例；`simple_hybrid_joint_controller*` 提供关节层控制器及调试工具；`wanren_arm` 与 `right_arm_*` 承担上肢硬件与 MoveIt! 支持。
- **命名规范**：所有关节以 `机器人名_肢体名_关节名` 形式发布到 `/joint_states`，便于在多控制板、多命名空间下统一管理。

## 环境要求
| 组件 | 说明 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 LTS（x86_64），建议禁用 `ModemManager` 以保证串口通信稳定 |
| ROS | ROS 1 Noetic，建议使用 `rosdep` 安装常用依赖 |
| 构建工具 | `catkin_tools`、`python3-catkin-tools`、`doxygen`、`liburdfdom-dev`、`libassimp-dev` 等 |
| 优化器 | [OCS2](https://leggedrobotics.github.io/ocs2/installation.html)，需额外安装 Boost ≥ 1.71、Eigen ≥ 3.3 以及 Raisim |
| 其他库 | `ros-noetic-serial`、`ros-noetic-joy`、`libserialport-dev`、`libglpk-dev`、`libglfw3-dev`、`onnxruntime 1.7` |

> **提示**：`onnxruntime` 建议安装到 `~/.local`，并在 `~/.bashrc` 中追加
> `export onnxruntime_DIR=~/.local` 与 `export LD_LIBRARY_PATH=~/.local/lib:${LD_LIBRARY_PATH}`。

## 快速开始
### 1. 准备 OCS2 工作空间（推荐 `~/ocs2_ws`）
1. 创建并初始化工作空间：
   ```bash
   mkdir -p ~/ocs2_ws/src && cd ~/ocs2_ws
   catkin init
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```
2. 克隆必要仓库（保持子模块）：
   ```bash
   cd src
   git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
   git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
   git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
   git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01
   git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git
   git clone https://github.com/ANYbotics/grid_map.git
   git clone https://github.com/leggedrobotics/ocs2.git
   ```
3. 按顺序编译，确保上一步无报错再执行下一步：
   ```bash
   catkin build elevation_mapping_cupy
   catkin build grid_map
   catkin build hpp-fcl
   catkin build pinocchio
   catkin build ocs2_robotic_assets
   catkin build ocs2    # 该步骤耗时最长
   ```
4. 将 `~/ocs2_ws/devel/setup.bash` 写入 `~/.bashrc` 或在需要时手动 `source`。

### 2. 准备本仓库（推荐 `~/catkin_ws`）
1. 创建工作空间并获取代码：
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://gitee.com/your-team/ludan_ws_ros1.git   # 以实际仓库地址为准
   ```
2. 安装依赖：
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src --rosdistro noetic -y
   ```
3. 初始化构建配置并编译：
   ```bash
   catkin init
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   catkin build
   ```
4. 将 `source ~/catkin_ws/devel/setup.bash` 写入 `~/.bashrc`，随后执行 `source ~/.bashrc` 使环境变量生效。

## 仓库结构概览
- `dmbot_serial`：统一的串口桥接节点，读取参数服务器中的 `motor_layout` 并与 STM32 电机驱动板通信，同时在 `/joint_states` 发布反馈。
- `simple_hybrid_joint_controller*`：不同肢体的 ros-control 控制器，提供 `command_one`、`command_same`、`command_matrix` 等话题接口。
- `legged_common` / `legged_hw` / `legged_examples`：核心算法、硬件接口与示例 Launch，用于连接 OCS2、状态估计与底层控制。
- `right_arm_hw`、`right_arm_moveit_config`、`wanren_arm`：上肢硬件接口与 MoveIt! 规划支持。
- `motor_control_gui4a.py` 等工具：串口连通性测试、单电机调试 GUI。

## 统一的电机命名与参数
### 基本参数
- `~robot_name`：机器人名称，默认 `ludan`。
- `~motor_layout`：电机布局列表。每个条目包含 `limb`、`joint`、`type`，并按照 CAN ID 顺序排列。
- `~num_motors`：仅在没有提供 `motor_layout` 时作为备选，用于指定电机总数。

### 默认 30 电机布局
```yaml
robot_name: ludan
motor_layout:
  - {limb: right_arm, joint: shoulder_yaw,   type: 10010l}
  - {limb: right_arm, joint: shoulder_pitch, type: 10010l}
  - {limb: right_arm, joint: shoulder_roll,  type: 10010l}
  - {limb: right_arm, joint: elbow_pitch,    type: 6248p}
  - {limb: right_arm, joint: wrist_pitch,    type: 4340}
  - {limb: right_arm, joint: wrist_roll,     type: 4340}
  - {limb: right_arm, joint: wrist_yaw,      type: 4340}
  - {limb: left_arm,  joint: shoulder_yaw,   type: 6248p}
  - {limb: left_arm,  joint: shoulder_pitch, type: 6248p}
  - {limb: left_arm,  joint: shoulder_roll,  type: 6248p}
  - {limb: left_arm,  joint: elbow_pitch,    type: 4340}
  - {limb: left_arm,  joint: wrist_pitch,    type: 4340}
  - {limb: left_arm,  joint: wrist_roll,     type: 4340}
  - {limb: left_arm,  joint: wrist_yaw,      type: 4340}
  - {limb: right_leg, joint: hip_yaw,        type: 10010l}
  - {limb: right_leg, joint: hip_roll,       type: 10010l}
  - {limb: right_leg, joint: hip_pitch,      type: 10010l}
  - {limb: right_leg, joint: knee_pitch,     type: 6248p}
  - {limb: right_leg, joint: ankle_pitch,    type: 4340}
  - {limb: right_leg, joint: ankle_roll,     type: 4340}
  - {limb: left_leg,  joint: hip_yaw,        type: 10010l}
  - {limb: left_leg,  joint: hip_roll,       type: 10010l}
  - {limb: left_leg,  joint: hip_pitch,      type: 10010l}
  - {limb: left_leg,  joint: knee_pitch,     type: 6248p}
  - {limb: left_leg,  joint: ankle_pitch,    type: 4340}
  - {limb: left_leg,  joint: ankle_roll,     type: 4340}
  - {limb: neck,      joint: yaw,            type: 4340}
  - {limb: neck,      joint: pitch,          type: 4340}
  - {limb: neck,      joint: roll,           type: 4340}
  - {limb: waist,     joint: yaw,            type: 10010l}
```
如未显式传入参数，`dmbot_serial` 会加载上述默认值。通过 `rosparam load` 或 Launch 文件可覆盖某些肢体或更换型号。

### 多控制板场景
1. 为每块控制板准备独立的 YAML：
   ```yaml
   # config/neck.yaml
   robot_name: ludan
   motor_layout:
     - {limb: neck, joint: pitch, type: 4340}
     - {limb: neck, joint: yaw,   type: 4340}
   ```
2. 以命名空间区分串口节点：
   ```bash
   rosparam load config/neck.yaml /neck_bridge
   roslaunch dmbot_serial test_motor.launch \
     port:=/dev/mcu_neck robot_name:=ludan __ns:=neck_bridge
   ```
3. 在上层控制器中根据需要 Remap `joint_states` 或编写聚合节点，实现多块控制板的数据整合。

### 指定 CAN ID 点动电机
1. 使用占位条目保证 YAML 中的顺序与硬件 CAN ID 对齐，例如仅控制 `/dev/mcu_neck` 的 CAN 7：
   ```yaml
   robot_name: ludan
   motor_layout:
     - {limb: neck, joint: reserve_0, type: 4340}
     - {limb: neck, joint: reserve_1, type: 4340}
     - {limb: neck, joint: reserve_2, type: 4340}
     - {limb: neck, joint: reserve_3, type: 4340}
     - {limb: neck, joint: reserve_4, type: 4340}
     - {limb: neck, joint: reserve_5, type: 4340}
     - {limb: neck, joint: reserve_6, type: 4340}
     - {limb: neck, joint: pitch,     type: 4340}
   ```
2. 启动硬件接口：
   ```bash
   roslaunch simple_hybrid_joint_controller_neck bringup_real.launch \
     port:=/dev/mcu_neck baud:=921600
   ```
3. 通过 `command_one` 话题下发命令：
   ```bash
   rostopic pub -r 10 /neck/all_joints_hjc_neck/command_one \
     std_msgs/Float64MultiArray "data: [7, 0.2, 0.0, 20.0, 1.0, 0.0]"
   ```
订阅 `/neck/joint_states` 可验证反馈是否来自目标电机。

## 控制信号链梳理
1. `backbringup_real.launch` 在指定命名空间下注入串口与控制循环参数，并包含 `legged_dm_hw.launch`。【F:simple_hybrid_joint_controller/launch/backbringup_real.launch†L1-L34】
2. `legged_dm_hw.launch` 生成 URDF、加载 `dm.yaml`，并启动硬件节点 `legged_dm_hw`。【F:legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L1-L20】【F:legged_examples/legged_dm_hw/config/dm.yaml†L1-L6】
3. `legged::DmHW` 初始化后订阅命令、IMU 与急停话题；其基类 `LeggedHW` 构造 `dmbot_serial::robot` 完成串口通信。【F:legged_examples/legged_dm_hw/src/legged_dm_hw.cpp†L46-L70】【F:legged_hw/src/LeggedHW.cpp†L15-L33】【F:dmbot_serial/src/robot_connect.cpp†L52-L163】
4. 控制器管理器加载 `joint_state_controller` 与 `AllJointsHybridController`，提供多种话题接口。【F:simple_hybrid_joint_controller/launch/backbringup_real.launch†L18-L33】【F:simple_hybrid_joint_controller/src/AllJointsHybridController.cpp†L1-L135】
5. `DmHW::write()` 将命令映射到串口帧，通过 `dmbot_serial::robot` 编码发送并接收反馈，最终发布到 `/joint_states`。【F:legged_examples/legged_dm_hw/src/DmHW.cpp†L42-L114】【F:dmbot_serial/src/robot_connect.cpp†L244-L420】

## 常见运行流程
### Gazebo 仿真
1. 启动仿真与控制器：
   ```bash
   roslaunch legged_controllers one_start_gazebo.launch
   ```
2. 在 `rqt_reconfigure` 中设置 `kp_position=100`、`kd_position=1`，在 Gazebo 中按 `Ctrl+Shift+R` 让机器人起立。
3. 通过手柄或 `rqt_robot_steering` 发布 `/cmd_vel`，并根据需求发布 `/reset_estimation`、`/load_controller` 与 `/set_walk` 等话题。

### 实机调试
1. 为 `/dev/ttyACM*` 设备授予权限并确认串口编号：
   ```bash
   ls /dev/ttyACM*
   sudo chmod a+rw /dev/ttyACM*
   ```
2. 启动实机控制：
   ```bash
   roslaunch legged_controllers one_start_real.launch
   ```
3. 在 `rqt_reconfigure` 中设置同样的 KP/KD，扶正机器人，随后按照仿真流程下达行走指令。

## 深入学习
- [基于 NMPC 和 WBC 的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630)
- [dm-control：ros-control 与达妙电机示例](https://gitee.com/xauter/dm-control)
- 相关开源项目：
  - https://github.com/bridgedp/hunter_bipedal_control
  - https://github.com/HighTorque-Robotics/livelybot_dynamic_control
  - https://github.com/qiayuanl/legged_control
- 参考文献：Flayols 等（2017）、Bloesch 等（2013）、Di Carlo 等（2018）、Grandia 等（2023）、Sleiman 等（2021）、Bellicoso 等（2016）、Kim 等（2019）。

## 致谢
感谢 leggedrobotics、ANYbotics 及社区贡献的开源资源，为本仓库的算法与工具提供了坚实基础。

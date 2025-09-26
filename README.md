# Ludan 双足机器人控制工作空间

## 仓库概览
- **系统定位**：面向 ROS 1 Noetic 的双足/多肢机器人控制栈，结合 ros-control 与 OCS2，实现从模型预测控制到硬件执行的一体化流程。【F:src/README.md†L3-L7】
- **串口桥接**：`dmbot_serial` 负责自动加载电机布局、打开串口、轮询反馈并持续发布 `/joint_states`，支持混合 30 路电机配置并在初始化阶段完成通信自检。【F:src/dmbot_serial/src/robot_connect.cpp†L100-L395】
- **硬件抽象层**：`legged_common` 提供混合关节接口，`legged_hw` 在初始化时挂接串口机器人对象并注册 joint/IMU/接触传感器句柄，`legged_examples` 则给出 `DmHW` 等具体实现与 Launch 流程。【F:src/legged_common/include/legged_common/hardware_interface/HybridJointInterface.h†L18-L124】【F:src/legged_hw/include/legged_hw/LeggedHW.h†L37-L74】【F:src/legged_hw/src/LeggedHW.cpp†L16-L52】【F:src/legged_examples/legged_dm_hw/src/DmHW.cpp†L20-L214】【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L1-L29】
- **关节控制器**：`ludan_joint_controller`（`simple_hybrid_joint_controller`）内置多种命令话题，允许通过 `command_one`、`command_matrix`、`command_moveJ` 等接口按需推送位置/速度/力矩指令。【F:src/ludan_joint_controller/src/AllJointsHybridController.cpp†L7-L165】【F:src/ludan_joint_controller/launch/bringup_real.launch†L1-L34】

## 目录速览
| 路径 | 作用 |
| --- | --- |
| `src/dmbot_serial` | 串口通信与电机命名规则，包含 `test_motor` 点动示例。 【F:src/dmbot_serial/src/robot_connect.cpp†L100-L395】【F:src/dmbot_serial/src/test_motor.cpp†L7-L80】|
| `src/legged_common` | 定义 `HybridJointInterface` 等扩展控制接口。 【F:src/legged_common/include/legged_common/hardware_interface/HybridJointInterface.h†L18-L124】|
| `src/legged_hw` | 将串口桥接包装成 ros-control 硬件接口。 【F:src/legged_hw/include/legged_hw/LeggedHW.h†L37-L74】【F:src/legged_hw/src/LeggedHW.cpp†L16-L52】|
| `src/legged_examples` | `DmHW` 等具体硬件节点、配置与 Launch，涵盖下肢/颈部示例。 【F:src/legged_examples/legged_dm_hw/src/DmHW.cpp†L20-L214】【F:src/legged_examples/legged_dm_hw/launch/legged_new.launch†L1-L31】|
| `src/ludan_joint_controller` | 关节级控制器插件及 bringup Launch。 【F:src/ludan_joint_controller/src/AllJointsHybridController.cpp†L7-L165】【F:src/ludan_joint_controller/launch/bringup_real.launch†L1-L34】|
| `src/wanren_arm` | WanRen 上肢 URDF，供 Launch 动态生成描述文件。 【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L5-L12】|
| `src/motor_control_gui4a.py` | Tkinter GUI，针对 ID7-13 提供速度/位置/力矩模式调试。 【F:src/motor_control_gui4a.py†L1-L153】|

## 核心功能模块
### 串口桥接：`dmbot_serial`
- 支持默认 30 路电机布局，并允许从参数服务器加载自定义 `motor_layout`；若缺失则回落到内置映射并记录名称去重情况。【F:src/dmbot_serial/src/robot_connect.cpp†L217-L303】
- 初始化时配置波特率、校验、线程与接收缓存，收发线程校验帧头/校验和后按电机类型解析反馈并发布 `sensor_msgs/JointState`。【F:src/dmbot_serial/src/robot_connect.cpp†L143-L399】
- `test_motor` 节点示范如何按 200 Hz 下发正弦命令，并打印部分反馈，便于快速验证硬件联通性。【F:src/dmbot_serial/src/test_motor.cpp†L7-L78】

### 硬件抽象与示例：`legged_common`、`legged_hw`、`legged_examples`
- `HybridJointHandle` 扩展了 ros-control 的关节句柄，允许一次写入位置、速度、PD 增益与前馈力矩，支撑混合控制策略。【F:src/legged_common/include/legged_common/hardware_interface/HybridJointInterface.h†L18-L124】
- `LeggedHW` 在 `init()` 中加载 URDF、实例化 `dmbot_serial::robot`、注册关节/IMU/接触传感器接口，为上层控制器提供统一入口。【F:src/legged_hw/include/legged_hw/LeggedHW.h†L37-L74】【F:src/legged_hw/src/LeggedHW.cpp†L16-L52】
- `DmHW` 节点继承 `LeggedHW`，额外订阅 IMU、手动覆盖与急停话题，按需选择手动/自动命令，将指令映射到串口命令帧并回显状态。【F:src/legged_examples/legged_dm_hw/src/DmHW.cpp†L20-L214】
- Launch 文件将 WanRen URDF、`dm.yaml` 参数与硬件节点结合，可通过 `ns` 区分多块控制板，并在 `legged_new.launch` 中展示双节点示例。【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L1-L29】【F:src/legged_examples/legged_dm_hw/config/dm.yaml†L1-L6】【F:src/legged_examples/legged_dm_hw/launch/legged_new.launch†L1-L31】
- 可执行程序 `legged_dm_hw` 使用 `LeggedHWLoop` 运行控制循环，并通过 `AsyncSpinner` 支持控制器动态加载。【F:src/legged_examples/legged_dm_hw/src/legged_dm_hw.cpp†L48-L80】

### 关节控制器：`ludan_joint_controller`
- `AllJointsHybridController` 自动加载关节列表，订阅 `command_same`、`command_pos_all`、`command_matrix`、`command_one`、`command_moveJ`，并在 `starting()` 与 `update()` 阶段写入混合命令。【F:src/ludan_joint_controller/src/AllJointsHybridController.cpp†L7-L163】
- `bringup_real.launch` 将硬件节点与控制器组合，生成命名空间化的 `joint_state_controller` 与 `AllJointsHybridController`，并允许重设串口端口/波特率。【F:src/ludan_joint_controller/launch/bringup_real.launch†L1-L34】
- `motor_control_gui4a.py` 通过 GUI 发布 `command_one`/`command_same`，支持速度/位置/力矩三种模式及一键急停，适合作为调试面板。【F:src/motor_control_gui4a.py†L5-L144】

## 电机命名与参数约定
- `robot_connect` 会按 `机器人名_肢体名_关节名` 生成唯一关节名，并记录默认布局；用户可通过 YAML 或命名空间覆盖布局，实现多控制板拓展或指定 CAN ID 点动。【F:src/dmbot_serial/src/robot_connect.cpp†L187-L275】【F:src/README.md†L79-L162】
- 在 Launch 中通过 `manual_topic_override` 与 `manual_cmd_timeout` 可以允许手动话题覆盖自动控制，便于安全调试。【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L9-L12】【F:src/legged_examples/legged_dm_hw/src/DmHW.cpp†L30-L208】

## 电机控制能力评估
### 摘要
- 串口桥节点自动配置 30 路电机、发布反馈并支持多类型解析，实现“接线即用”的底层接口。【F:src/dmbot_serial/src/robot_connect.cpp†L217-L369】
- 硬件抽象层将串口电机包装成 ros-control 资源，Launch 文件提供从 URDF 加载到硬件节点启动的一键流程。【F:src/legged_hw/include/legged_hw/LeggedHW.h†L37-L74】【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L1-L15】【F:src/legged_examples/legged_dm_hw/src/legged_dm_hw.cpp†L48-L80】
- `AllJointsHybridController` 通过多种话题接口桥接上位控制需求，配合 GUI/脚本即可按需下发单关节或批量命令。【F:src/ludan_joint_controller/src/AllJointsHybridController.cpp†L42-L165】【F:src/motor_control_gui4a.py†L21-L144】

### 构建与运行可行性
- README 已列出操作系统、OCS2 依赖、`catkin_tools` 及 `rosdep` 步骤，但当前容器缺少 `catkin_make` 等 ROS 工具，无法直接编译验证。【F:src/README.md†L9-L69】【c72d3a†L1-L2】
- 包含的 `package.xml` 未声明对 `serial` 库的依赖，`rosdep` 无法自动拉取 `ros-noetic-serial`，需要手动补充。【F:src/dmbot_serial/package.xml†L31-L60】
- 当依赖满足后，Launch 会加载 WanRen URDF、`dm.yaml` 控制参数并启动硬件节点，结合手动覆盖参数可实现稳定 bringup。【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L5-L15】【F:src/legged_examples/legged_dm_hw/config/dm.yaml†L1-L6】

### 支撑“launch + topic”控制的组件
1. `dmbot_serial` 串口桥接：发现电机、解析反馈并发送复合命令。【F:src/dmbot_serial/src/robot_connect.cpp†L100-L399】
2. `legged_common` 混合关节接口：暴露位置/速度/增益/力矩指令槽位。【F:src/legged_common/include/legged_common/hardware_interface/HybridJointInterface.h†L18-L124】
3. `legged_hw` 基类：加载 URDF、注册硬件接口并持有串口机器人实例。【F:src/legged_hw/include/legged_hw/LeggedHW.h†L37-L74】【F:src/legged_hw/src/LeggedHW.cpp†L16-L52】
4. `legged_examples/legged_dm_hw`：实现读写循环、手动覆盖逻辑与 Launch 配置，发布命令/反馈回显话题。【F:src/legged_examples/legged_dm_hw/src/DmHW.cpp†L20-L214】【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L1-L29】
5. `ludan_joint_controller` 插件：提供 `command_*` 话题驱动 `HybridJointInterface`。【F:src/ludan_joint_controller/src/AllJointsHybridController.cpp†L42-L163】
6. 调试工具：`test_motor` CLI 与 `motor_control_gui4a.py` GUI，方便在硬件或上位机侧发布命令验证反馈。【F:src/dmbot_serial/src/test_motor.cpp†L7-L78】【F:src/motor_control_gui4a.py†L21-L144】

### 风险与后续工作
- 在部署前需安装 README 所列 ROS/OCS2 依赖，并确认容器或目标主机已具备 `catkin_tools` 等构建工具。【F:src/README.md†L9-L69】
- 建议在 `dmbot_serial/package.xml` 中补充 `<depend>serial</depend>`，确保 `rosdep` 自动拉取串口库。【F:src/dmbot_serial/package.xml†L31-L60】
- 启动前检查 URDF/串口参数与真实硬件一致，必要时通过 GUI/`command_one` 小幅度验证，避免因布局不匹配导致误动作。【F:src/legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L5-L15】【F:src/motor_control_gui4a.py†L83-L144】

## 开发环境准备
1. **OCS2 工作空间**：在 `~/ocs2_ws` 依次克隆 pinocchio、hpp-fcl、ocs2 等仓库，并按照顺序 `catkin build`，最后 `source ~/ocs2_ws/devel/setup.bash`。【F:src/README.md†L21-L49】
2. **本仓库工作空间**：在 `~/catkin_ws` 克隆代码后使用 `rosdep` 安装依赖，配置 `catkin` 并执行 `catkin build`，最后在 `~/.bashrc` 中 `source` 构建结果。【F:src/README.md†L51-L69】

## 常见运行流程
- **Gazebo 仿真**：`roslaunch legged_controllers one_start_gazebo.launch`，在 `rqt_reconfigure` 中设置 KP/KD，再通过手柄或话题控制机器人行走。【F:src/README.md†L171-L179】
- **实机调试**：授予串口权限后执行 `roslaunch legged_controllers one_start_real.launch`，并在 `rqt_reconfigure` 设定增益以安全扶正机器人。【F:src/README.md†L180-L190】

## 进一步阅读
- 推荐的博客、示例项目与参考文献已在 `src/README.md` 汇总，可帮助理解 NMPC、WBC 及社区实现。【F:src/README.md†L192-L199】

## 致谢
感谢 leggedrobotics、ANYbotics 及开源社区提供的算法与工具资源，使本仓库得以构建完整的双足机器人控制流程。【F:src/README.md†L201-L202】

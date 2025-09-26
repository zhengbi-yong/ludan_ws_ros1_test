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

## 串口电机测试实操指南
以下步骤可帮助你在不同控制板（如 `/dev/mcu_neck`、`/dev/mcu_leftarm` 等）上逐一验证各个 CAN ID 电机的联通性。流程覆盖硬件接线、参数准备、启动测试与结果判定四个阶段。

### 1. 硬件准备
1. **接线与供电**：确认末端 STM32 控制板通过 USB-C/Type-B 与上位机直连，并在上电前检查 48 V/24 V 总线与电机驱动器接线牢靠。
2. **禁用串口抢占服务**：Ubuntu 20.04 默认启用 `ModemManager`，可能导致串口被占用，建议在机器人主机上执行 `sudo systemctl disable --now ModemManager.service`。仓库在环境要求中也推荐禁用该服务以提升串口稳定性。【F:src/README.md†L9-L18】
3. **用户权限与串口命名**：
   - 把当前用户加入 `dialout` 组：`sudo usermod -a -G dialout $USER && newgrp dialout`。
   - 重新插拔控制板后执行 `dmesg | tail` 或 `ls /dev/ttyACM*` 确认出现的新串口节点，依据控制板用途重命名到统一的 `/dev/mcu_*` 规则（常用做法是在 `/etc/udev/rules.d/99-ludan.rules` 中根据串口的 `idVendor`/`idProduct`/序列号创建软链接）。
   - 通过 `sudo chmod a+rw /dev/mcu_*` 临时赋予可读写权限，确保后续节点能正常访问串口设备。【F:src/README.md†L171-L187】

### 2. 参数与布局确认
1. **默认布局**：`dmbot_serial::robot` 若未读取到参数服务器上的 `motor_layout`，会自动落回 30 路默认布局，并按 `机器人名_肢体名_关节名` 生成唯一的关节名称。【F:src/dmbot_serial/src/robot_connect.cpp†L217-L305】
2. **自定义布局文件**：为每块控制板新建一个 YAML（例如 `config/neck.yaml`），按照 CAN ID 顺序枚举需要激活的关节；若要跳过某些 ID，可保留占位条目保持索引一致，示例可参考 `src/README.md` 中的配置段落。【F:src/README.md†L97-L160】
3. **加载参数**：在启动测试前执行 `rosparam load config/neck.yaml /neck_bridge` 将布局写入对应命名空间，确保 `robot_connect` 初始化时能读取到目标布局。【F:src/README.md†L118-L135】

### 3. 启动测试节点
1. **创建专属命名空间**：为避免多块板子的反馈互相覆盖，建议为每个串口分配独立命名空间（例如 `neck_bridge`、`leftarm_bridge`），并在 Launch 中设置 `__ns` 与 `port` 参数。
2. **使用 `test_motor` 节点**：该节点默认以 200 Hz 推送正弦指令，并在控制台打印部分电机反馈，适合快速验证串口链路。可以直接调用现有 Launch：
   ```bash
   roslaunch dmbot_serial test_motor.launch \
     port:=/dev/mcu_neck __ns:=neck_bridge robot_name:=ludan
   ```
   其中 `port` 指向目标控制板串口，`robot_name` 会拼接进关节命名；需要切换到左臂控制板时，只需改成 `port:=/dev/mcu_leftarm __ns:=leftarm_bridge` 即可。【F:src/dmbot_serial/launch/test_motor.launch†L1-L8】【F:src/dmbot_serial/src/robot_connect.cpp†L112-L169】【F:src/dmbot_serial/src/test_motor.cpp†L7-L78】
3. **调整测试幅度**：为确保安全，`test_motor` 默认零幅值输出。若要小角度摆动某个 CAN ID，可在命令行附加参数 `_pos_amp:=0.05 _kp:=2.0 _kd:=0.1` 或在 Launch 文件中修改对应 `<param>`，幅值与增益会传入 `fresh_cmd_motor_data()` 并经串口下发。【F:src/dmbot_serial/src/test_motor.cpp†L14-L49】

### 4. 验证反馈与常见排查
1. **观察串口日志**：若串口被占用或波特率设置错误，`robot_connect` 会在初始化时抛出 “Unable to open motor serial port” 日志，需要重新检查 udev 配置或波特率参数。【F:src/dmbot_serial/src/robot_connect.cpp†L143-L181】
2. **检查 `joint_states`**：在对应命名空间下运行 `rostopic echo /neck_bridge/joint_states`，应能看到名称包含 `robot_name_肢体_关节` 的条目；若只有零值或数量不符，需确认 YAML 中的条目与实际 CAN ID 一一对应。【F:src/dmbot_serial/src/robot_connect.cpp†L187-L268】
3. **点动单个电机**：可在加载完布局后，通过 `rostopic pub` 向 `/neck_bridge/all_joints_hjc_neck/command_one` 发送命令，仅指定目标索引的关节进行微小动作，示例命令可参考 `src/README.md` 中的点动流程。【F:src/README.md†L136-L160】
4. **GUI 辅助调试**：对于需要频繁切换模式的电机，可运行 `motor_control_gui4a.py`，它会向 `command_one`/`command_same` 发布话题，并提供一键急停按钮以防突发情况。【F:src/motor_control_gui4a.py†L21-L144】

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

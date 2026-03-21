# MiniHumanoid_ROS2

MiniHumanoid 的 ROS2 控制工程，包含：

- `MotCtrl_Can`：底层 CAN 电机控制节点（DM 电机）
- `robot_control`：上层机器人控制节点（订阅传感器与关节反馈，发布关节命令）

---

## 1. 环境要求

- 操作系统：Linux（建议 Ubuntu）
- ROS2：建议 Humble（或与你本机一致的 ROS2 发行版）
- CAN：SocketCAN（`can0`~`can3`）
- 编译工具：`colcon`、`ament_cmake`

> 注意：本工程依赖 Linux SocketCAN，不能直接在 Windows 原生环境运行电机控制。

---

## 2. 目录说明

```text
MiniHumanoid_ROS2/
├─ MotCtrl_Can/                 # CAN 电机控制包
│  ├─ launch/MotCtrl.launch.py  # 电机控制节点启动文件
│  └─ src/                      # 电机节点与驱动源码
├─ robot_control/robot_control/ # 上层控制包
│  └─ src/                      # 上层控制节点源码
├─ can.sh                       # 配置 can0~can3（1Mbps）
└─ mot_setzero.sh               # 电机零点相关 CAN 指令脚本
```

---

## 3. 构建工程

在 `MiniHumanoid_ROS2` 目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果你是新终端，运行节点前都需要重新 `source install/setup.bash`。

---

## 4. 运行步骤

### 步骤 1：使能四路 CAN

```bash
sudo ./can.sh
```

可用下面命令检查接口状态：

```bash
ip -details link show can0
ip -details link show can1
ip -details link show can2
ip -details link show can3
```

### 步骤 2：启动电机 CAN 控制节点

```bash
ros2 launch MotCtrl_Can MotCtrl.launch.py
```

### 步骤 3：启动机器人控制节点

```bash
ros2 run robot_control robot_control
```

---

## 5. 主要 ROS2 接口

### `MotCtrl_node`

- 订阅：
  - `/joint_command_left_leg`
  - `/joint_command_right_leg`
  - `/joint_command_left_arm`
  - `/joint_command_right_arm`
- 发布：
  - `/joint_states_left_leg`
  - `/joint_states_right_leg`
  - `/joint_states_left_arm`
  - `/joint_states_right_arm`
- 消息类型：`sensor_msgs/msg/JointState`

### `robot_control`

- 订阅：
  - `/cmd_vel`（`geometry_msgs/msg/Twist`）
  - `/Imu`（`sensor_msgs/msg/Imu`）
  - 四肢 `/joint_states_*`
- 发布：
  - 四肢 `/joint_command_*`（`sensor_msgs/msg/JointState`）

---

## 6. CAN 与电机分配

- `can0`：左腿（ID 1~3）
- `can1`：右腿（ID 4~6）
- `can2`：左臂（ID 7~9）
- `can3`：右臂（ID 10~12）

默认波特率由 `can.sh` 配置为 `1000000`。

---

## 7. 常用调试命令

查看当前 ROS2 话题：

```bash
ros2 topic list
```

查看左腿关节反馈：

```bash
ros2 topic echo /joint_states_left_leg
```

查看左腿控制命令：

```bash
ros2 topic echo /joint_command_left_leg
```

监听 CAN 总线（示例）：

```bash
candump can0
```

---

## 8. 电机零点脚本

执行前请确认：

- CAN 接口已经正常 up
- 总线上仅连接目标电机，避免误操作

示例：

```bash
bash mot_setzero.sh
```

---

## 9. 常见问题

### 1) `can0: No such device`

说明接口未创建或驱动未加载。请先检查硬件和系统 CAN 配置。

### 2) 节点能启动但电机不动作

- 检查 `can.sh` 是否执行成功
- 检查电机 ID 与代码配置是否一致
- 检查 `robot_control` 是否在发布 `/joint_command_*`

### 3) 收不到 `/joint_states_*`

- 检查 `MotCtrl_node` 是否正在运行
- 检查 CAN 通信是否正常（`candump canX`）

---

## 10. 快速开始（最短路径）

```bash
cd MiniHumanoid_ROS2
colcon build --symlink-install
source install/setup.bash
sudo ./can.sh
ros2 launch MotCtrl_Can MotCtrl.launch.py
# 新开一个终端后：
source install/setup.bash
ros2 run robot_control robot_control
```

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

可选：启动时覆盖 `MotCtrl_node` 关键参数（示例）

```bash
ros2 launch MotCtrl_Can MotCtrl.launch.py \
  kp_mit:=1.0 \
  kd_mit:=0.1 \
  cmd_timeout_ms:=200 \
  watchdog_period_ms:=20 \
  control_period_ms:=5 \
  can0_start_id:=0 can0_motor_count:=3 \
  can1_start_id:=0 can1_motor_count:=3 \
  can2_start_id:=0 can2_motor_count:=3 \
  can3_start_id:=0 can3_motor_count:=3 \
  can0_joint_dirs:="[1,-1,1]" \
  can1_joint_dirs:="[1,-1,1]" \
  can2_joint_dirs:="[1,-1,1]" \
  can3_joint_dirs:="[1,-1,1]"
```

说明：`MotCtrl.launch.py` 已声明这些 launch 参数，既可按上面方式传入，也可不传使用默认值。

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

- `can0`：左腿（默认 ID 0~2）
- `can1`：右腿（默认 ID 0~2）
- `can2`：左臂（默认 ID 0~2）
- `can3`：右臂（默认 ID 0~2）

默认波特率由 `can.sh` 配置为 `1000000`。

---

## 7. `MotCtrl_node` 参数说明

- `kp_mit`：MIT 控制中的 Kp，默认 `0.0`
- `kd_mit`：MIT 控制中的 Kd，默认 `0.0`
- `cmd_timeout_ms`：命令超时阈值，超过后进入安全输出，默认 `300`
- `watchdog_period_ms`：watchdog 检查周期，默认 `20`
- `control_period_ms`：固定频率控制循环周期，默认 `5`（约 200Hz）
- `can0_start_id/can1_start_id/can2_start_id/can3_start_id`：每条 CAN 链路的起始电机 ID，默认 `0`
- `can0_motor_count/can1_motor_count/can2_motor_count/can3_motor_count`：每条 CAN 链路电机数量，默认 `3`
- `can0_joint_dirs/can1_joint_dirs/can2_joint_dirs/can3_joint_dirs`：每条 CAN 链路的关节方向（整型数组），默认 `[1,-1,1]`

说明：
- 订阅回调只缓存目标，不直接下发电机命令。
- 实际控制由 `control_period_ms` 定时循环执行。
- 超时后 watchdog 下发安全命令（保持当前位置、速度 0、力矩 0）。
- 每个 limb 的 `JointState` 维度按该 limb 实际电机数量动态处理（不再固定 3 关节）。
- 命令消息要求至少包含该 limb 的关节数量，超过部分会被忽略。
- 电机 ID 按每条 CAN 独立配置，不要求全局连续编号。
- 方向参数长度若与电机数量不一致，节点会自动补齐或截断并打印告警。

---

## 8. 控制链路时序（文字版）

1. 上层节点（如 `robot_control`）发布四肢命令到 `/joint_command_*`。
2. `MotCtrl_node` 的订阅回调只做两件事：校验消息维度 + 缓存目标值。
3. 固定周期 `control_timer_callback` 读取缓存目标，统一调用 `MotorMitModeCmd(...)` 下发。
4. 同一个控制周期内发布 `/joint_states_*`，反馈当前电机状态。
5. `watchdog_timer_callback` 持续检查“距上次命令的时间”。
6. 若超时（`elapsed > cmd_timeout_ms`），切换到安全输出：保持当前位置、速度 0、力矩 0。
7. 命令恢复后，退出超时状态，重新进入正常周期控制。

这套结构的核心目的是把“消息接收”和“电机下发”解耦，减少回调抖动对控制时序的影响。

---

## 9. 常用调试命令

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

## 10. 调参建议（起步值）

> 以下仅作为联调起点，实际值请根据负载、减速比、机构刚度和控制目标逐步调整。

| 场景 | kp_mit | kd_mit | control_period_ms | cmd_timeout_ms | 建议 |
|---|---:|---:|---:|---:|---|
| 首次上电（空载） | 0.2 ~ 0.8 | 0.02 ~ 0.08 | 5 ~ 10 | 200 ~ 400 | 先确认方向和零位，动作幅度小 |
| 单关节联调 | 0.8 ~ 2.0 | 0.05 ~ 0.2 | 5 | 200 | 先调到无明显振荡，再加大 kp |
| 多关节低速联动 | 1.0 ~ 3.0 | 0.1 ~ 0.3 | 5 | 150 ~ 250 | 同步观察电流峰值与发热 |
| 保守稳定优先 | 0.5 ~ 1.5 | 0.08 ~ 0.2 | 5 ~ 8 | 200 ~ 300 | 适合长时间连续测试 |

调参顺序建议：

1. 固定 `control_period_ms`（建议先用 `5`）。
2. 从低 `kp_mit` 开始，逐步上调到“刚好跟随且不抖”。
3. 用 `kd_mit` 抑制振荡，避免过大导致动作发硬。
4. 适当收紧 `cmd_timeout_ms`，确保上位机中断时能快速进入安全模式。

---

## 11. 电机零点脚本

执行前请确认：

- CAN 接口已经正常 up
- 总线上仅连接目标电机，避免误操作

示例：

```bash
bash mot_setzero.sh
```

---

## 12. 常见问题

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

## 13. 快速开始（最短路径）

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

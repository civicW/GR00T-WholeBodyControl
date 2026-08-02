# SONIC 数据集字段说明（含 DM 夹爪）

> 面向 GR00T 模型训练同学。说明 `run_data_exporter.py` 采集的 LeRobot 数据集里**每个字段**的含义、形状与来源，重点标注 DM 夹爪新增项与训练 loss 相关的 mask。
>
> 最后更新：2026-08-01｜字段定义：[`gear_sonic/data/features_sonic_vla.py`](../gear_sonic/data/features_sonic_vla.py)（`get_features_sonic_vla()`），写出：[`gear_sonic/data/exporter.py`](../gear_sonic/data/exporter.py)（`Gr00tDataExporter`）。

## 0. 概述

- **格式**：LeRobot 数据集（`parquet` + `mp4`），含标准索引列 `index / frame_index / timestamp / episode_index / task_index / task`。
- **采集频率**：默认 50 Hz（`--data-collection-frequency`）。
- **机器人**：Unitree G1，**43 DoF = 29 身体 + 7 左手 + 7 右手**。
- **本体/动作维度来源**：43 维配置向量由 `body_q(29) + left_hand_q(7) + right_hand_q(7)` 组装，顺序见 `_JOINT_GROUPS_FOR_STATE`。

## 1. 启用 DM 夹爪的前提

录到夹爪数据需要**同时**满足：

1. exporter 带开关：`--end-effector dm_gripper`（否则下文 D 节 4 列不存在）。
2. 机器人侧（Jetson）运行 CAN 桥 `gripper_teleop_bridge.py`，把 DM 电机位置以 `ee_feedback`（ZMQ 端口 **5558**）发回 PC。
3. PC 与机器人网络互通：PC `enp0s31f6` 同时在 `192.168.123.0/24`（DDS/夹爪反馈）与 `192.168.1.0/24`（相机/SSH）；`rp_filter=0`。

## 2. 字段总表（按模态分组）

### A. 图像 `observation.images.*`（video / mp4）
| 字段 | shape | 说明 |
|---|---|---|
| `ego_view` | [480,640,3] | 主相机（默认只录这一个） |
| `left_wrist` / `right_wrist` | [480,640,3] | 仅 `--record-wrist-cameras` 时 |

### B. 本体状态（来自 deploy 的 `g1_debug` @ ZMQ 5557）
| 字段 | dtype | shape | 说明 |
|---|---|---|---|
| `observation.state` | float64 | (43,) | 全身关节位置 `whole_q`（身体 + 手） |
| `observation.eef_state` | float64 | (14,) | 左右手腕位姿：pos(3)+quat(4) ×2（FK 计算） |
| `observation.root_orientation` | float64 | (4,) | 基座朝向四元数 (qw,qx,qy,qz) |
| `observation.projected_gravity` | float64 | (3,) | 重力投影（由朝向推得） |
| `observation.cpp_rotation_offset` | float64 | (4,) | C++ 端旋转偏移 |
| `observation.init_base_quat` | float64 | (4,) | 初始基座朝向 |

### C. 动作
| 字段 | dtype | shape | 说明 |
|---|---|---|---|
| `action.wbc` | float64 | (43,) | 实际下发的关节目标（= `last_action` + 双手 `last_action`） |
| `action.motion_token` | float64 | (64,) | 运动 token 状态 |

### D. 🆕 末端执行器（DM 夹爪）—— 仅 `--end-effector dm_gripper`
| 字段 | dtype | shape | 说明 |
|---|---|---|---|
| `action.gripper` | float32 | (2,) | **期望**归一化闭合度 [0=开, 1=闭]，[left, right] |
| `action.gripper_angle` | float32 | (2,) | **期望**电机角度（度），[left, right] |
| `observation.gripper` | float32 | (2,) | **实际**归一化位置（CAN 读回），[left, right] |
| `observation.gripper_angle` | float32 | (2,) | **实际**电机角度（度），[left, right] |

> 夹爪**不**并入 `observation.state`（仍 43 维）、**不**并入 `action.wbc`（仍 43 维），是**独立的 4 列**。
> 角度范围：`ANGLE_OPEN = -150°`，`ANGLE_CLOSE = -90°`；归一化 `norm = (angle - open) / (close - open)`。

### E. 遥操作辅助信号 `teleop.*`（来自 PICO 的 pose/planner @ ZMQ 5556）
| 字段 | dtype | shape | 说明 |
|---|---|---|---|
| `teleop.smpl_joints` | float32 | (72,) | SMPL 关节 |
| `teleop.smpl_pose` | float32 | (63,) | SMPL 姿态 |
| `teleop.body_quat_w` | float32 | (4,) | SMPL 身体朝向 |
| `teleop.target_body_orientation` | float32 | (6,) | 目标身体朝向（rot6d，yaw 归一化） |
| `teleop.left_hand_joints` / `right_hand_joints` | float32 | (7,) | 遥操手部目标 |
| `teleop.left_wrist_joints` / `right_wrist_joints` | float32 | (3,) | 手腕 roll/pitch/yaw（取自 G1 joint_pos） |
| `teleop.stream_mode` | int32 | (1,) | 遥操流模式 |
| `teleop.planner_mode` | int32 | (1,) | 步态模式 |
| `teleop.planner_movement` / `facing` | float32 | (3,) | planner 速度/朝向向量 |
| `teleop.planner_speed` / `height` | float32 | (1,) | planner 速度/高度 |
| `teleop.vr_3pt_position` | float32 | (9,) | VR 三点位置（左腕/右腕/颈 xyz） |
| `teleop.vr_3pt_orientation` | float32 | (18,) | VR 三点朝向（每点 rot6d） |
| `teleop.delta_heading` | float64 | (1,) | 航向变化 |
| `teleop.smpl_frame_index` | int64 | (1,) | SMPL 帧计数 |

### F. 元信息 / 索引
`task`（语言提示，来自 `--task-prompt`）、`index`、`frame_index`、`timestamp`、`episode_index`、`task_index`。

## 3. 给训练同学的 3 个关键点

1. **夹爪是独立模态，别和 43 维本体混在一起。** 夹爪的 action/observation 各自是 (2,) 的 `gripper` / `gripper_angle`，训练时要单独作为一组 action/observation 维度处理。`observation.state` 与 `action.wbc` 里的 14 维（7 左手 + 7 右手）是 **Dex3 手指位**，**不是** DM 夹爪——见下条。

2. **`meta/info.json` 里有 `ee_config` + mask（影响 loss）。** 由 `run_data_exporter.py` 写出：
   ```
   ee_config = {
     "type": "dm_gripper",
     "feature_prefix": "gripper",
     "dims": 2,
     "angle_range": [-150.0, -90.0],
     "action_mask": [...43...],   # 14 个 Dex3 手指位为 False
     "state_mask":  [...43...]    # 同上
   }
   ```
   `action_mask` / `state_mask` 把 43 维里的 **14 个 Dex3 手指位标 `False`**——因为这台机器用的是 DM 夹爪、Dex3 手是惰性的，训练时 action/state loss **不应**拟合这 14 维（否则会被拉向 0、污染身体维）。**训练 loader 必须按这个 mask 过滤。**

3. **归一化约定。** `action.gripper` / `observation.gripper` ∈ [0,1]（0 全开、1 全闭）；`*_angle` 是原始电机角度（度）。模型输出夹爪动作后，部署端按 `angle = open + norm*(close - open)` 换算成 CAN 目标。

## 4. 数据来源与链路（排查字段缺失用）

| 数据 | 来源 | 链路 |
|---|---|---|
| 本体/动作（B、C） | C++ deploy | `g1_debug` → ZMQ 5557 → exporter 解析（`run_data_exporter.py` `_assemble_proprio` 一带） |
| 遥操信号（E） | `pico_manager_thread_server.py` | pose/planner → ZMQ 5556 → exporter |
| 夹爪**实际**位置（D 的 obs） | `gripper_teleop_bridge.py`（Jetson）读 DM4310 CAN 反馈 | `ee_feedback` → ZMQ 5558 → exporter（~50 Hz） |
| 夹爪**期望**值（D 的 action） | teleop 流的 `left_gripper` / `right_gripper` | 直接从 pose/planner 取 |

**CAN 寻址**（`gripper_teleop_bridge.py` 参数）：
- 左夹爪：USB2CANFD dongle `0`，电机 CAN ID `0x02`（反馈 ID `0x12`）
- 右夹爪：USB2CANFD dongle `1`，电机 CAN ID `0x01`（反馈 ID `0x11`）
- 总线：classic CAN 2.0 @ 1 Mbps；MIT 模式闭环（kp=10, kd=0.5）；写入 ~1 kHz，反馈 ~50 Hz。

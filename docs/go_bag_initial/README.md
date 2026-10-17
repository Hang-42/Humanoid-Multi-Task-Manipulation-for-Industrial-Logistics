# V55 + Linker Hand 示范初始目标

用户于 2026-10-17 确认 `data/灵巧手go.bag` 的采集平台为 **V55 + Linker Hand**。
本次离线读取 bag，提取初始化终点；随后已将 `poses.chest_high` 和 `end_effector.initial`
补入 `configs/robots/v55_linker_hand.json`，未发送机器人动作。用户后续要求过渡参数参考 V53、限幅使用默认值，
并指定实机联调默认通过；当前 V55 已按该用户确认启用，详见 [工作站环境与模型说明](../WORKSTATION_ENV_AND_MODELS.md)。

## 可采用的目标值

双臂顺序为左臂关节 1–7、右臂关节 1–7，单位为 **度**：

```json
{
  "poses": {
    "chest_high": [
      -10.236, 8.632, 3.001, -94.519, -29.750, -6.363, 14.860,
      -22.051, -6.009, 6.231, -80.351, 27.464, 13.436, 8.091
    ]
  },
  "end_effector": {
    "initial": [0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0]
  }
}
```

每侧手部顺序为 `thumb, thumb_aux, index, middle, ring, pinky`。
手部数值是厂商兼容 ROS 接口的 **0–100 命令单位**，不是角度。
每侧 `[0,100,0,0,0,0]` 中的 `100` 对应 `thumb_aux` 侧摆通道；手指屈曲通道均为 `0`。
通过现有 Linker 兼容驱动使用这些值，驱动内部负责手型映射。

## 数据依据

bag 长度约 65.443 秒，SHA256：
`f9279a28bd017988753535c707f53b9ff7da5de6a3389d682e1beba15caf2714`。

选择开头首次指令改变前的共同区间：bag 记录时间偏移 `[0.207321405, 1.743265152)` 秒。

| 消息 | 区间内帧数 | 核对结果 |
|---|---:|---|
| `/kuavo_arm_traj` | 177 | 14 维目标完全一致，取该指令目标 |
| `/control_robot_hand_position` | 177 | 左右各 6 维，目标完全一致 |
| `/sensors_data_raw` | 1105 | 29 关节中索引 13–26，弧度转为度；位置最大峰峰值约 0.02239° |
| `/dexhand/state` | 336 | 按名称重排后，12 通道反馈均与目标一致 |

双臂实测反馈与指令目标最大绝对差约 **1.496°**。目标采用原始指令，未把跟踪偏差叠加成新的目标。
反馈速度字段的最大瞬时值约 5.320°/s；这里的位置稳定性依据位置峰峰值，不将该片段描述成所有速度读数均为零。
bag 记录时间与各消息 Header 时间存在约 25 秒偏移，以上按 bag 记录时间选取初始保持片段，
不是对不同消息的精密硬件时间同步验证。

厂商控制源码 `humanoidController.cpp` 的 `/kuavo_arm_traj` 回调会把位置乘以 `π/180`，
与本次弧度实测反馈相符，确认该话题虽使用 `JointState` 消息，位置单位仍为度。

已用备份的 `biped_s55.urdf` 检查目标终点关节限位：无越界，距最近限位约 25.140°。
URDF SHA256 为 `c76db1a077cda62b0310bd86c9371daf813468cb54c28ddc4a8fdf4c3c43101b`。
现有项目中的 `poses.chest_high` 与本次提取值在保留三位小数后完全相同。

## 使用范围

这组数值补充的是 **站立完成后、任务推理开始前的双臂与手部终点**，可以作为 V55 配置中
`poses.chest_high` 与 `end_effector.initial` 的数据依据。bag 没有给出从任意起始姿态到此终点的安全路径，
也没有定义项目的垂臂判定范围、侧举/折臂中间姿态、十通道抓握限幅或现场碰撞空间。
它不改变目标机器人自身的标定参数，不代表整个 V55 配置已完成实机验证。

简明目标文件为 [`initialization_targets.json`](initialization_targets.json)，它是目标片段，不是完整部署配置。
精确浮点数、反馈中位数、统计、消息清单与源标定消息见 [`targets.json`](targets.json)。
原始首帧图像：
[头部](cam_h_first.jpg)、[左腕](cam_l_first.jpg)、[右腕](cam_r_first.jpg)。

提取脚本位于工作空间根目录 `tools/extract_bag_initial_targets.py`，需要 Python、NumPy 和 rosbags。
本次使用已有隔离依赖，复现命令（在工作空间根目录执行）：

```bash
python tools/extract_bag_initial_targets.py "data/灵巧手go.bag" \
  --rosbags-dir tmp/rosbags \
  --urdf tmp/eef-compat-reference-20261016/v55/lower/v55-model/urdf/biped_s55.urdf \
  --output "Humanoid Multi-Task Manipulation for Industrial Logistics/docs/go_bag_initial/targets.json"
```

已在当前 Python 环境安装 rosbags 时可以省略 `--rosbags-dir`；`--urdf` 省略时只提取数据，不检查限位。

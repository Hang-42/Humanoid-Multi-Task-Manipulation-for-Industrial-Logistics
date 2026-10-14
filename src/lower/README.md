# PI05 下位机 ROS 包

这是运行在 `/home/lab/kuavo-ros-opensource` ROS Noetic 工作空间旁的 catkin package。`package.xml` 和 `CMakeLists.txt` 用于 ROS 包发现；实际入口是同目录的 shell/Python 脚本。这里的桥接实现以 2026-10-13 Qi 工作站部署源码为基线，保留其动作限幅、状态时效、控制器占用与回执检查；本次只调整三路相机订阅命名空间和 ROS 节点名。

## 文件与数据流

| 文件 | 职责 |
| --- | --- |
| `lower_start.sh` | 加载 ROS Noetic 和乐聚下位机工作空间，设置 ROS 主站/本机地址及工作站地址，前台启动桥接节点。 |
| `robot_bridge.py` | 订阅图像和机器人状态、请求工作站推理；收到动作后执行预检、限幅、控制器回执检查，并发布双臂和手部目标。仅 `--execute` 才可申请控制权和发送运动命令。 |
| `robot_initialize.sh` | 工作站入口；每次将 `initialize/` 整包传到下位机 `/home/lab/kuavo_pi05_lower/initialize/`，校验后运行其中的 `robot_local.sh`。默认仅检查，不自动触发站立。 |
| `initialize/` | `robot_local.sh`、`robot_controller.sh` 及四个 Python 依赖，完成控制器启动、分阶段站立和双臂初始位姿。 |
| `camera_observations.py`、`camera_codec.py`、`observation.py` | 图像解码/方向校正、机器人状态组装、三路 JPEG 与 16 维状态打包。 |
| `protocol.py` | 与工作站 PI05 proxy 通信的 `SVSHAD01` TCP 帧及有界收发。 |
| `action_safety.py`、`arm_ack.py` | 动作约束、控制器连接和轨迹回执校验。 |

上位机发布 `sensor_msgs/CompressedImage`：`/pi05_camera/head_cam_h/compressed`、`/pi05_camera/wrist_cam_l/compressed`、`/pi05_camera/wrist_cam_r/compressed`。桥接还订阅 `/sensors_data_raw` (`kuavo_msgs/sensorsData`) 和 `/dexhand/state` (`sensor_msgs/JointState`)。状态按 `q[13:20], l_thumb/100, q[20:27], r_thumb/100` 组成 16 维；三路图像与状态发送到工作站 `192.168.26.20:19092`。工作站返回动作块；桥接检查后向 `/kuavo_arm_target_poses` 和 `/control_robot_hand_position` 发布命令，并监听 `/humanoid_mpc_target_arm` 控制器回执。

## 下位机运行

本包依赖已安装的 `/opt/ros/noetic` 和 `/home/lab/kuavo-ros-opensource/devel`；`initialize/` 随本包维护，工作站运行 `robot_initialize.sh` 时直接传输至下位机。默认 `ROS_MASTER_URI=http://192.168.26.1:11311`、`ROS_IP=192.168.26.1`。可分别通过 `LOWER_ROS_MASTER_URI`、`LOWER_ROS_IP`、`PI05_WORKSTATION_HOST`、`PI05_WORKSTATION_PORT` 修改。桥接参数也可直接传 `--host`、`--port`。

```bash
bash lower_start.sh --check                  # 下位机只检查本地环境
```

在 Qi 工作站的项目根目录运行初始化入口（会传输 `initialize/`）：

```bash
bash src/lower/robot_initialize.sh --check   # 初始化只读预检
bash src/lower/robot_initialize.sh --prepare # 分阶段准备
# 由现场人员按初始化运行时要求确认机器人已手动放低
bash src/lower/robot_initialize.sh --confirm-lowered
bash src/lower/robot_initialize.sh --execute # 显式站立及双臂初始位姿
bash src/lower/robot_initialize.sh --status
```

下位机桥接仍单独以前台启动：

```bash
bash lower_start.sh --steps 5                # 只读推理查询，不发送动作
bash lower_start.sh --execute --steps 5 --duration 60  # 现场看护下才运行
```

`--execute` 前还必须确认工作站 proxy 可用、三路图像及机器人反馈新鲜、机器人站立且处于已初始化位姿。桥接保持前台运行；断链、过期数据、控制器状态异常或终端结束时会停止发送新动作并请求双臂保持。代码未在本次重构中上机执行，不能把静态检查视为实机安全验证。当前手部安全与初始化标定取自已有部署，换任务/末端硬件前需要单独核验。

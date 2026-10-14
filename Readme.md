# Humanoid Multi-Task Manipulation for Industrial Logistics

以下命令均从 Qi 工作站的项目根目录 `/data/Humanoid Multi-Task Manipulation for Industrial Logistics` 执行。机器人站立和双臂初始化会使实机运动；仅在现场人员确认安全、急停状态及运动空间后执行相应步骤。本 README 记录命令，不会自动启动机器人。

## 1. 机器人站立与双臂初始化

`robot_initialize.sh` 每次都会把 `src/lower/initialize/` 整包传到下位机，再运行其中的 `robot_local.sh`。`--check` 只做预检；`--prepare` 进行站立前准备；现场人员手动放低机器人并核对位置后，才可确认并执行站立和双臂初始位姿。

```bash
bash src/lower/robot_initialize.sh --check
bash src/lower/robot_initialize.sh --prepare
# 现场确认双脚距地约 10 cm、周围无人且无障碍后：
bash src/lower/robot_initialize.sh --confirm-lowered
bash src/lower/robot_initialize.sh --execute
bash src/lower/robot_initialize.sh --status
```

## 2. 实时读取三路相机图像

先检查并部署上位机相机中继（`--start` 会通过 SSH 将 `src/upper/` 传到上位机并启动相机驱动及中继）。

```bash
bash src/deploy_pi05.sh --check
bash src/deploy_pi05.sh --start
```

在上位机 ROS 环境中分别查看三路压缩图像的实时帧率；如需并行观察，可在三个终端分别执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.12
rostopic hz /pi05_camera/head_cam_h/compressed
rostopic hz /pi05_camera/wrist_cam_l/compressed
rostopic hz /pi05_camera/wrist_cam_r/compressed
```

上述三路话题类型为 `sensor_msgs/CompressedImage`。`rostopic hz` 验证持续出帧；它不显示画面。如需查看图像内容，可用 ROS 图像查看器订阅对应的 compressed 话题。

## 3. 部署 PI05 推理

在 Qi 工作站选定一个场景模型（`cz`、`kd12`、`lj`、`lp`）；下面以 `kd12` 为例。检查模型权重与推理环境；首次配置环境时执行安装，然后以前台启动服务：

```bash
bash src/deploy_pi05.sh --pi05-install
bash src/deploy_pi05.sh --pi05-check --model kd12
bash src/deploy_pi05.sh --pi05-start --model kd12 --gpu 0 --port 8000
```

已有环境可略过 `--pi05-install`。另开终端查看端口状态：

```bash
bash src/deploy_pi05.sh --pi05-status --model kd12 --port 8000
```

此入口仅启动 OpenPI WebSocket 推理服务，默认绑定 `127.0.0.1:8000`，不会发送机器人动作。下位机桥接使用 `192.168.26.20:19092` 的 TCP 协议，二者目前不是同一个接口；接入桥接前仍需适配 proxy，不能仅凭推理服务启动就执行机器人动作。

# Humanoid Multi-Task Manipulation for Industrial Logistics

以下命令均在工作站工作空间`Humanoid Multi-Task Manipulation for Industrial Logistics/` 执行：

## 1. 选择机器人配置

选择机器人配置：

```bash
# V53 + Revo1
ROBOT_PROFILE="configs/robots/v53_revo1.json"

# V55 + Revo1
ROBOT_PROFILE="configs/robots/v55_revo1.json"

# V55 + lejuclaw
ROBOT_PROFILE="configs/robots/v55_lejuclaw.json"
```


## 2. 机器人站立与双臂初始化

先检查配置和下位机环境：

```bash
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE" --check
```

该命令会传输初始化程序及配置，然后做预检，不发送站立或手臂运动指令

现场准备完成后执行交互初始化：

```bash
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE"
```

屈腿完成后在同一终端输入小写 `o` 并按 Enter 才继续

## 3. 安装及检查 PI05 推理环境

首次安装环境：

```bash
bash src/deploy_pi05.sh --pi05-install
```

检查所选机器人和任务的环境与模型配置：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-check --model kd12
```

支持任务 `kd12`、`cz`、`lj`、`lp`

## 4. 实机运行 PI05 推理
完成初始化后实机运行 PI05 推理：


```bash
bash src/deploy_pi05.sh \
  --robot-profile "$ROBOT_PROFILE" \
  --pi05-live --model kd12 \
  --steps 10 --sync-grasp --grasp-lookahead 5 \
  --duration 600
```
不传时长时持续运行，直到 Ctrl+C

启动动作前仍需在终端输入 `o` 并按 Enter

恢复到对应机器的初始化姿态：
```bash
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --execute
```

## 5.三路图像可视化

相机中继启动后，在工作站图形桌面通过 X11 转发登录当前上位机：

```bash
ssh -X leju_kuavo@192.168.26.12
```

在上位机终端运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.12
rosrun image_view image_view image:=/pi05_camera/head_cam_h _image_transport:=compressed __name:=head_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_l _image_transport:=compressed __name:=left_wrist_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_r _image_transport:=compressed __name:=right_wrist_image_view &
wait
```

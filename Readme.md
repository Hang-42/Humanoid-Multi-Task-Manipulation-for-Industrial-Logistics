# Humanoid Multi-Task Manipulation for Industrial Logistics

以下命令均在 Linux 工作站的项目根目录`Humanoid Multi-Task Manipulation for Industrial Logistics/` 执行，并在同一终端保留配置变量


## 1. 选择机器人配置


### 1.1 工作站SSH公钥写入机器人上下位机
没有 SSH 密钥时生成；
```bash
ssh-keygen -t ed25519
```

写入机器人上下位机：
```bash
ssh-copy-id lab@192.168.26.1
ssh-copy-id leju_kuavo@192.168.26.12
```

### 1.2 识别机器人配置
工作站有线连接机器人上下位机后，先运行只读识别脚本：

```bash
python3 src/detect_robot.py \
  --lower lab@192.168.26.1 \
  --upper leju_kuavo@192.168.26.12 \
  --batch
```
输出示例：

```text
机器人配置：V53 + lejuclaw
```

### 1.3 声明机器人、任务配置和工作站有线IP

```bash
export PI05_WORKSTATION_IP="192.168.26.108"  # 换成实际有线 IP
ROBOT_PROFILE="configs/robots/v55_linker_hand.json"
MODEL="kd12"
```
支持任务参数 `kd12`、`cz`、`lj`、`lp`



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

屈腿完成后，按终端提示人工降低机器人，再在同一终端输入小写 `o` 并按 Enter 才继续。


## 3.实机运行 PI05 推理
### 3.1 安装及检查 PI05 推理环境
```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-install
```

检查所选机器人和任务的环境与模型配置：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-check --model "$MODEL"
```



### 3.2 实机运行 PI05 推理
完成初始化和环境检查后，先执行不发送机器人动作的模型查询：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-query --model "$MODEL" --steps 10
```

该查询仍会部署/启动相机、通信桥及推理服务。查询成功后，在交互终端运行实机推理：

```bash
bash src/deploy_pi05.sh \
  --robot-profile "$ROBOT_PROFILE" \
  --pi05-live --model "$MODEL" \
  --steps 10 --sync-grasp --grasp-lookahead 5 \
  --duration 600
```
不传时长时持续运行，直到 Ctrl+C

启动动作前仍需在终端输入 `o` 并按 Enter


### 3.3 恢复到对应机器的初始化姿态
现场确认空手、姿态满足恢复条件后执行；仍需输入 `o` 并按 Enter：

```bash
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --execute
```

该操作只恢复双臂，不重新站立，也不张开或复位末端

## 4.三路图像可视化

相机中继启动后，另开工作站图形终端，通过 X11 转发登录当前上位机:
```bash
ssh -X leju_kuavo@192.168.26.12
```

在上位机终端运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.12
unset ROS_HOSTNAME
rosrun image_view image_view image:=/pi05_camera/head_cam_h _image_transport:=compressed __name:=head_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_l _image_transport:=compressed __name:=left_wrist_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_r _image_transport:=compressed __name:=right_wrist_image_view &
wait
```

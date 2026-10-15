# Humanoid Multi-Task Manipulation for Industrial Logistics

## 1. 机器人站立与双臂初始化

在工作站的项目根目录 `Humanoid Multi-Task Manipulation for Industrial Logistics` 运行：

```bash
bash src/lower/robot_initialize.sh
```

完成屈腿准备后停在终端等待现场人员将机器人降低至双脚距地约 10 cm

同一终端输入小写 `o` 并按 Enter，脚本才会继续一次站立及双臂初始化

## 2. 部署 PI05 推理

机器人应完成站立与双臂初始化后部署 PI05 推理,下位机底层控制器由初始化流程启动

### 2.1 下载并检查 PI05 推理环境

首次部署时，从 `pi05/openpi` 安装推理环境并检查 `kd12` 模型：

```bash
./src/deploy_pi05.sh --pi05-install
./src/deploy_pi05.sh --pi05-check --model kd12
```
### 2.2 启动 PI05 推理

在现场人员值守、急停可立即操作、机器人稳定站立且运动范围内无人和障碍时，运行：

```bash
./src/deploy_pi05.sh --pi05-live --steps 5
```

`--steps` 可设置为 1–50，省略时默认执行 2 步

`--model` 默认场景是 `kd12`，其他场景可加 `--model cz`、`--model lj` 或 `--model lp`

例如 `./src/deploy_pi05.sh --pi05-live --model cz --steps 5`
启动前会检查相机中继是否已注册到当前 ROS master，以及三路话题是否有实时帧；检测到本项目管理的旧中继失效时会自动重启该相机会话。

按 Ctrl+C 停止并请求双臂保持。

### 2.3 恢复初始化位置
```bash
bash src/recover/recover_start.sh --execute
```


## 3. 在工作站实时可视化三路相机图像

相机中继已启动后，在工作站的图形桌面终端通过 X11 转发显示三路画面

通过SSH登录上位机，开启X11 图形界面转发:
```bash
ssh -X leju_kuavo@192.168.26.12
```
启动 image_view程序:
```bash
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.12
rosrun image_view image_view image:=/pi05_camera/head_cam_h _image_transport:=compressed __name:=head_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_l _image_transport:=compressed __name:=left_wrist_image_view &
rosrun image_view image_view image:=/pi05_camera/wrist_cam_r _image_transport:=compressed __name:=right_wrist_image_view &
wait
```

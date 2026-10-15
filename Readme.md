# Humanoid Multi-Task Manipulation for Industrial Logistics

## 1. 选择机器人配置

以下命令均在工作站工作空间`Humanoid Multi-Task Manipulation for Industrial Logistics/` 执行：

支持三份配置：

| 配置 | 用途 | 当前状态 |
| --- | --- | --- |
| `configs/robots/v53_revo1.json` |V53 灵巧手机器人， | 默认配置；兼容旧的无身份字段记录 |
| `configs/robots/v55_revo1.json` | V55 + 强脑 Revo1 灵巧手 | 待填写、待实机验证的模板 |
| `configs/robots/v55_lejuclaw.json` | V55 + 乐聚两指夹爪 | 待填写、待实机验证的模板 |

选择机器人配置：

```bash
# V53 + Revo1
ROBOT_PROFILE="configs/robots/v53_revo1.json"

# V55 + Revo1
ROBOT_PROFILE="configs/robots/v55_revo1.json"

# V55 + 两指夹爪
ROBOT_PROFILE="configs/robots/v55_lejuclaw.json"
```

上面三行是三个选项，只执行需要的一行。不传 `--robot-profile` 时默认使用 V53 配置。

V55 模板的 `validated` 默认是 `false`，标定、初始姿态、相机序列号和模型路径等字段保留为 `null`。模板未补齐时，启动脚本会在联网或调用 ROS 之前退出，不能直接用于实机。

先查看缺少的配置：

```bash
python3 -B src/common/robot_profile.py --robot-profile "$ROBOT_PROFILE" --describe --model kd12
```

需要逐台填写并核实：

| 字段 | 内容 |
| --- | --- |
| `robot_id` | 唯一的实机编号，只用字母、数字、下划线和连字符 |
| `network` | SSH 用户/地址、上下位机和 Qi 的机器人网口 IP、代理和推理端口 |
| `paths` | ROS 工作空间、部署根目录、该机型及末端对应 URDF、URDF SHA256、官方配置路径 |
| `calibration` | 该台机器自己的标定文件及 SHA256，不能复用 V53 或另一台机器的标定 |
| `joints` | 实际传感器关节数、按左臂7/右臂7排列的索引、腰部索引 |
| `poses` | side、folded、chest_high 的14维角度（度），以及垂臂姿态识别区间 |
| `tool` | URDF 末端固定关节名称模板，及相对此关节的左右 TCP 平移（米）、RPY（弧度） |
| `safety` | 手臂速度、跟踪误差和持续时间、初始姿态容差、恢复路径包络 |
| `end_effector` | 类型对应的初始值、开合映射、速度/电流/超时和 Revo1 手指限幅 |
| `cameras` | 序列号、源话题、驱动旋转与送入模型前的旋转；核对最终模型图像 |
| `models.<任务>` | 匹配该机器人/末端的数据契约、模型配置名、asset、checkpoint，以及模型验证状态 |

`validated=true` 是现场核验后的明确记录，不会自动完成硬件验证。先完成参数确认和必要实机验证，再填写这个字段。每个需要运行的任务，还须填写对应 `models.<任务>.validated=true`。

下位机必须预先具有支持 V55 的官方 ROS1 控制器和对应消息。官方 `kuavo.json` 的 `EndEffectorType` 应与配置一致：Revo1 使用匹配的 `qiangnao` 驱动，两指夹爪使用 `lejuclaw`。本项目只检查匹配关系，不会自动改写官方驱动配置、标定或 URDF。

当前相机启动文件适用于 RealSense 双腕相机和 Orbbec Gemini 头相机，输入仍为848×480、三路 RGB。其他相机型号需要另行调整驱动 launch。不调用旧的 Revo1 FULL 服务，也不启用 Turbo。

当前 V53 的 `end_effector.current_setup=revo1_advanced_can` 保留 Qi 最新的 Revo1 Advanced 抓力初始化：在启动全身控制器前，通过已有 CAN 工具将双手五个抓握电机的保护电流设为600 mA（原值500 mA），逐指读回；拇指辅助轴保持原值。失败时尝试恢复已改参数并阻止控制器启动。仅当前上电周期有效。部署会核对当前开机的 `hand_force.json`，并打印 `DEXHAND_GRIP_CURRENT_CONFIGURED`；旧的 FULL 服务被该驱动忽略，不能据此判断真实握力。

如需恢复500 mA，只能在机器人可靠悬吊、控制器尚未启动的初始化窗口执行：

```bash
PI05_GRIP_CURRENT_MA=500 bash src/lower/robot_initialize.sh --robot-profile configs/robots/v53_revo1.json
```

该参数是保护电流，不等于实测夹持力提高20%；不要在站立时停控制器改参数。两份 V55 配置使用 `current_setup=vendor`，由对应官方驱动提供经现场确认的末端参数；现有 CAN 工具只适用于已核查的当前 V53，V55 配置不能直接启用它。

配置补齐后做本地校验（不会连接或启动机器人）：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --profile-check
python3 -B src/common/robot_profile.py --robot-profile "$ROBOT_PROFILE" --model kd12
```

第一条检查硬件配置字段，第二条额外检查所选任务的模型配置字段；不会读取远程标定文件或自动证明模型实机效果。未通过时退出码为2，并列出问题。

## 2. 机器人站立与双臂初始化

先检查配置和下位机环境：

```bash
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE" --check
```

该命令会传输初始化程序及配置，然后做预检，不发送站立或手臂运动指令。

现场准备完成后执行交互初始化：

```bash
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE"
```

当前初始化流程保留“屈腿准备 → 现场确认人工降低 → 一次站立 → 双臂及末端初始化”。脚本提示双脚距地约10 cm；V55 使用此流程前，须确认与目标机型的官方操作流程和现场支撑条件一致。

准备完成后在同一终端输入小写 `o` 并按 Enter 才继续。Revo1 使用对应配置的12通道目标；夹爪使用左右两个目标，并等待实际完成反馈。初始化会核对机器人版本、官方末端类型、标定哈希和（V55 必填的）URDF 哈希。

查看当前启动的记录：

```bash
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE" --status
```

初始化记录、轨迹检查点和运行后保持记录均绑定机器编号、机型、末端类型、配置哈希、boot ID 和控制器实例。完成初始化后不要直接修改配置继续运行。配置改变后需要重新核验记录；不能删除一次站立记录绕过限制。

历史 S53 的相机试探动作在 V55 上禁用；V55 应填写经过验证的初始化姿态。

## 3. 安装及检查 PI05 推理环境

首次安装环境：

```bash
bash src/deploy_pi05.sh --pi05-install
```

检查所选机器人和任务的模型环境：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-check --model kd12
```

支持任务 `kd12`、`cz`、`lj`、`lp`，提示词保持与现有微调任务一致。checkpoint 路径可以相对项目根目录，也可以使用绝对路径。checkpoint 的参数、数据处理配置和 `norm_stats.json` 必须匹配；不能只替换归一化统计来把灵巧手模型变成夹爪模型。

模型状态/动作仍是16维：

```text
左臂7（弧度） + 左末端闭合比例1 + 右臂7（弧度） + 右末端闭合比例1
```

Revo1 保持与现有训练一致的拇指反馈标量，执行时展开为多手指指令并保留辅助拇指。夹爪按各自标定的开/闭位置映射为0～1。两种末端使用各自验证的模型，维度相同不代表已有权重能直接迁移。

## 4. 相机、只读推理及实机执行

仅安装/启动相机链路（要求目标 ROS master 已存在；不启动机器人控制器）：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --start
```

完成初始化后，可用一条命令准备相机、推理服务、代理和下位机桥接，查询一次模型并打印动作诊断，结束前不发送机器人动作：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-query --model kd12 --steps 5 --retime
```

此命令会安装本项目发布文件、启动所需的相机/推理进程，并由既有推理启动保护关闭已确认的厂商 VR 手臂发布节点。它不会发送模型动作，也不会站立或重启底层控制器。

现场检查完成后，运行有时长限制的动作：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 5 --retime --duration 30
```

不传时长时持续运行，直到 Ctrl+C：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 5 --retime
```

启动动作前仍需在终端输入 `o` 并按 Enter。Ctrl+C 请求双臂进入保持模式，并取消夹爪尚未发送的目标；不会自动张开末端。已经送入夹爪驱动的运动可能继续完成，不能把取消待发送目标当作夹爪急停。

当前执行层将手臂速度限制为 `min(10°/秒, profile 中的 arm_speed_deg_s)`，在10 Hz下每段不超过1°；普通执行和 `--retime` 均适用。模型权重、末端闭合决策及600 mA配置不变。为避免现有初始化/模型服务身份失效，此次降速不修改机器人profile及其哈希。

动作接管后，首窗口必须收到控制器本次模式切换的单点回执，后续窗口从上一窗口已确认的控制器终点连续规划，不再从滞后的实测位置重新起步。收到执行回执后会逐点核对起点、中间点、终点、时间和速度；缺少参考或回执不一致会停止，不重发轨迹。手臂10°／0.2秒跟踪保护保持不变。

双手同步抓取的受控测试（可选，不与 `--retime` / approach / single-step 组合；上电并初始化后再运行）：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 10 --sync-grasp --grasp-lookahead 5 --duration 300
```

`--sync-grasp` 当前仅适用于12通道Revo1，按 `min(10°/秒, profile 上限)` 同步播放，每次最多5秒。左右手首次闭合或明显加大闭合时，均切断旧预测中之后的手臂动作；双臂须在5°误差内、位置变化率不超过5°/秒持续0.3秒，才发送模型闭合目标。变化率使用至少120毫秒的单调时钟窗口内的位置总变化计算，不再将传感器位置差除以异步到达的MPC时钟差。3秒内不到位仍停止，不改用实测位置覆盖目标，不自动重试；`GRASP_ARM_SETTLE_PROGRESS/TIMEOUT` 会记录具体关节误差、变化率和超时图像。

`--grasp-lookahead 0..10` 必须配合 `--sync-grasp`，默认0。显式设为5时，仅当首次模型闭合点位于前10步之外、后5步之内，才把本窗口延长至该闭合点；最多15个模型帧，仍受5秒播放预算和总时限约束。不会跳过中间帧、补造闭合或执行闭合后的旧抬臂动作。已闭合阶段，小于5单位的模型目标抖动保留最近已执行目标，避免99/100反复等待；累计到5单位的加大目标会重新进入等待。

闭合后至少等待1秒；每只待闭合手的五指反馈总变化范围须不超过2单位并持续0.4秒，再满足：五指均在目标10单位以内（`target_reached`）；或本次闭合阶段中拇指及至少两根其余手指各有至少8单位实测闭合进展且无明显超调（`closure_motion_settled`）。双手同时闭合须两边都通过。重新张开会清除该手的进展参考；反馈中断超过0.25秒重计稳定时间，预期闭合的手指反向回退超过5单位会停止。3秒未完成则停止保持，不抬臂。

结束等待后要求三路相机、关节和手部状态均有新样本，再重新推理（最多等1秒）。开闭决策仍来自模型，保留已执行目标仅用于上述去抖；手指限幅、AUX保持值、600mA配置、profile身份和手臂10°/0.2秒跟踪保护不变。这不是抓稳检测，日志始终为 `contact_verified=false`、`grasp_verified=false`。10月16日上一轮现场反馈流程基本跑通，但反复尝试且最终因双臂到位超时停止；停止后还观测到右臂第7关节速度/位置反馈不一致，不能据此排除硬件或遥测异常。此次双手等待、去抖和前瞻仅完成离线验证；上电后须重新初始化、核对遥测并另行确认现场安全，再进行一轮有时限测试。

常用参数：

| 参数 | 含义 |
| --- | --- |
| `--robot-profile PATH` | 同一次初始化、推理、恢复使用同一份配置 |
| `--model kd12/cz/lj/lp` | 选择任务模型，默认 kd12 |
| `--steps 1..50` | 每次预测执行的模型帧数；一键入口默认2 |
| `--gpu N` | 推理使用的 GPU 编号，默认0；复用已有服务时也必须一致 |
| `--retime` | 保留手臂模型目标，通过增加10 Hz控制点满足配置的速度限制 |
| `--sync-grasp` | 两手闭合前到位检查、闭合等待和新观测；不是抓稳检测 |
| `--grasp-lookahead 0..10` | 仅sync-grasp；默认0，显式延长至邻近的模型闭合点 |
| `--duration 1..600` | 限制本轮执行时长（秒）；省略则持续运行 |
| `--single-step` | 只执行一个有界的0.1秒手臂目标，末端保持；不能与 retime/approach 组合 |
| `--approach-only` | 保持末端初始目标，执行到首个模型闭合位置；必须配合 retime |
| `--approach-extra-steps 0..10` | approach-only 模式下额外跟随的模型帧数 |
| `--resume-current` | 现场检查后显式接纳有限范围内的当前姿态；不绕过其他检查 |

单步示例：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --single-step
```

保持末端并接近抓取位置：

```bash
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 50 --retime --approach-only --approach-extra-steps 0
```

## 5. 恢复到对应机器的初始化姿态

先只预演：

```bash
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --check
```

现场确认后执行：

```bash
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --execute
```

恢复使用当前实机反馈、所选 URDF/TCP 和对应配置的初始化目标；末端不发开合命令。关节/FK 包络检查不等于完整环境碰撞检测；未通过的路径不会执行。

## 6. 两台 V55 的完整命令示例

Revo1 机器（先补齐并核验配置）：

```bash
ROBOT_PROFILE="configs/robots/v55_revo1.json"
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --profile-check
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE"
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-query --model kd12 --steps 5 --retime
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 5 --retime --duration 30
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --check
```

两指夹爪机器（先补齐并核验配置）：

```bash
ROBOT_PROFILE="configs/robots/v55_lejuclaw.json"
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --profile-check
bash src/lower/robot_initialize.sh --robot-profile "$ROBOT_PROFILE"
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-query --model kd12 --steps 5 --retime
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --pi05-live --model kd12 --steps 5 --retime --duration 30
bash src/recover/recover_start.sh --robot-profile "$ROBOT_PROFILE" --check
```

逐条执行并检查结果，不要一次粘贴整组命令。两个模板不能通过照抄当前 V53 的标定、姿态、工具参数后直接置为已验证。

同一 Qi 同时连接两台机器人时，配置独立的机器人网段/ROS master，并使用不同的 `robot_id`、`proxy_port` 和 `inference_port`，例如19092/8000与19093/8001。两台下位机同时使用同一网段的相同 IP 会产生冲突。会话按机器人编号和模型命名；服务身份和配置哈希不匹配时拒绝复用已有监听进程。

升级前已运行的旧推理/代理没有配置哈希，新脚本会拒绝复用。先在机器人保持且无动作运行时检查 `tmux ls`，由操作者停止对应旧推理/代理，再使用新命令启动；本次源码修改不会主动替换运行中的服务。

## 7. 离线代码验证

以下测试不启动 ROS 节点或发送机器人动作：

```bash
python3 -B -m unittest discover -s src/common -p test_profiles.py
python3 -B -m unittest discover -s src/lower -p test_lower_contract.py
python3 -B -m unittest discover -s src/lower -p test_hand_force_setup.py
python3 -B -m unittest discover -s src/recover -p test_recover_plan.py
```

这些测试验证配置隔离、末端映射、夹爪命令排队、状态/动作契约和规划边界。通过测试不代表 V55 的标定、工具参数或任务成功率已完成实机验证。

## 8. 当前 V53 三路图像可视化

相机中继启动后，在 Qi 图形桌面通过 X11 转发登录当前上位机：

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

V55 使用其配置中对应的上位机用户、工作空间和 ROS 网络地址替换本节的 V53 地址。

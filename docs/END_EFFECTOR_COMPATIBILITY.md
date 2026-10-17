# 末端执行器兼容

本次在 `3ecec71 version1.1` 和工作区已有的 V53 夹爪现场适配基础上，加入
V55 Linker Hand。原有 Revo1 配置保持默认，原有现场夹爪配置与未提交修改保留。
本次只读访问备份并修改本地代码，未部署到机器人、启动控制器或发送运动指令。

## 配置与通信

| 末端 | 配置 | 厂商 EndEffectorType | ROS 命令 | ROS 反馈 |
|---|---|---|---|---|
| V53 Revo1 | `configs/robots/v53_revo1.json` | `qiangnao` | `/control_robot_hand_position`，`kuavo_msgs/robotHandPosition` | `/dexhand/state`，`sensor_msgs/JointState` |
| V53 lejuclaw | `configs/robots/v53_lejuclaw.json` | `lejuclaw` | `/control_robot_leju_claw`，`kuavo_msgs/controlLejuClaw` 服务 | `/leju_claw_state`，`kuavo_msgs/lejuClawState` |
| V55 Linker Hand | `configs/robots/v55_linker_hand.json` | `linker_hand` | `/control_robot_hand_position`，`kuavo_msgs/robotHandPosition` | `/dexhand/state`，`sensor_msgs/JointState` |

Linker 的 `eef_type` 为 `linker_hand`，`end_effector.interface` 为
`kuavo_dexhand_6dof`。这是备份中的乐聚兼容接口，不表示支持所有 Linker SKU，
也不接入厂商原生 `/cb_*` 话题、0～255 数据或其他自由度接口。

模型输入、输出仍为 16 维：左臂 7 个弧度关节角、左末端闭合比例、右臂 7 个
弧度关节角、右末端闭合比例。通信适配不会自动验证现有 checkpoint、训练动作语义
或归一化统计能否跨末端使用，任务模型仍有独立的 `validated` 检查。

## 原机备份依据

两份来源均在 `mingzi@192.168.49.17`：

- V53 夹爪：`/data/kuavo-host-backup-20261016-155214`。
- V55 Linker：`/data/kuavo-v55-dexhand-backup-20261016-jDHJbi/v55-linker-hand`；
  补充控制器源码及版本审计来自它的父备份目录。

V55 厂商源码版本记录为 `a3e877726e074ed7b0bf988f0e3a7ad91e6edd11`，备份有未提交修改。
`lower/v55-config/kuavo.json` 明确选择双 `linker_hand`。旧初始化包中的
V53/Revo1 配置不能用作这台 V55 的部署配置。

不仅检查了 `linker-hand-mapping.h` 和 `touch-hand-controller.h` 的声明，还对备份的
`libros_hardware_plant_cxx.so` 做了只读符号及调用点检查：

- `DexhandController::send_position` 调用 `forwardMapDualHandsInplace`。
- `DexhandRosNode::publish_loop` 调用 `inverseMapThumbFlexion` 和
  `inverseMapThumbAbduction`。
- 库内包含 `control_robot_hand_position`、`dexhand/command`、`dexhand/state`
  话题及 `l_thumb`、`l_thumb_aux`、`l_pinky`、`r_thumb` 等兼容通道名。
- `robotHandPosition.msg` 为左右各一个 `uint8[]`；兼容控制器采用左右各 6 个
  0～100 位置值。上层沿用这一量程，不重复执行 Linker 手型映射。

V55 模板记录了备份中的 URDF、标定文件和该桥接库的 SHA256。下位机预检会核对
这些文件，驱动变化后应重新确认接口再更新配置。文件摘要及库调用证据保存在
[`end_effector_backup_audit.json`](end_effector_backup_audit.json)。

`HandProtocolType.ini` 为 `proto_can`，但备份设备清单仍有 `revo1_hand` 标签。
这些标签不覆盖 V55 的 `EndEffectorType`，也不允许使用 Revo1 独占 CAN 电流工具。

## 动作映射与初始化

Revo1 和 Linker 的 ROS 兼容通道顺序均为左、右各
`thumb, thumb_aux, index, middle, ring, pinky`。模型反馈取左右 thumb/100；
模型输出驱动每侧 thumb/index/middle/ring/pinky，使用该配置自己的 10 个
`finger_caps`，保留启动时的两个 thumb_aux。Linker 的 `initial` 来自用户确认的 V55 `go.bag`；
`finger_caps` 按用户要求设为十通道各 `50` 的工程默认值，含义是位置限幅，不是握力百分比。

两类灵巧手均使用已有的 12 通道受控渐变初始化、命名反馈和新鲜度检查。
Linker 使用 `current_setup=vendor`，不要求 Revo1 电流回执；
`revo1_advanced_can` 仅允许原已核实的 V53 Revo1 使用。

夹爪沿用服务异步发送、单个在途请求、最新目标覆盖待发目标及真实到位状态检查。
模型比例通过每侧独立的 `open_position`、`closed_position` 线性映射，允许方向相反。
当前 V53 现场配置继续使用已确认的张开 `[10.5,10.5]` 和软件闭合参考 `[95,95]`；
后者不是本次验证的成功抓握位置。备份消息/头文件的通用注释写有相反的开合方向，
不能据此直接反转已有的现场测量配置；另一台夹爪应重新确认自己的开合参考。

## V55 配置状态与检查

`v55_linker_hand.json` 已从备份恢复上下位机地址、三路相机序列号和 URDF 工具参考变换，
并记录厂商默认臂姿及关节顺序。用户随后要求实机联调默认通过，现已据此启用 V55 配置和四个模型；
IP 环境变量与实际权重映射见 [工作站环境与模型说明](WORKSTATION_ENV_AND_MODELS.md)。逐项来源见 [备份配置恢复说明](ROBOT_PROFILE_BACKUP_RECONCILIATION.md)。
使用时：

- 声明 `PI05_WORKSTATION_IP`；已恢复的机器人地址与相机序列号供运行时核对。
- 12 通道初始手型已由 V55/Linker `go.bag` 提取；抓握通道默认限幅各为 50，任务抓握效果待验证。
- 手臂过渡姿态与安全阈值已参考 V53/Revo1 填入并通过 V55 URDF 离线抽样检查；物理手部尺寸、路径净空与跟踪仍需现场确认，详见 [默认配置说明](V55_LINKER_DEFAULTS.md)。
- 四个任务 checkpoint 与资产映射已按用户选定的工作站目录填写；运行入口仍检查实际模型文件与接口。

备份可以确定通信与部分文件事实，不能补出尚未验证的抓握限幅和实机成功率。
查看缺项不需要 ROS，不连接机器人：

```bash
python3 src/common/robot_profile.py \
  --robot-profile configs/robots/v55_linker_hand.json --describe
```

填写且完成验证后，现有初始化、推理及恢复入口均通过 `--robot-profile` 选择该配置。
未完成配置时，Shell 入口会在 SSH、SCP、tmux、ROS 启动等外部操作前退出。
V55 不接受旧 V53 初始化/保持记录；不同配置哈希的记录也不能混用。

## 离线回归

```bash
python3 -B -m unittest discover -s src/common -p 'test_*.py'
python3 -B -m unittest discover -s src/lower -p 'test_*.py'
```

`test_linker_hand.py` 覆盖独立身份、模板阻断、消息通道和 uint8 数值、反馈乱序与过期、
独立限幅和辅助通道保留、渐变初始化选择，以及不复用 Revo1 CAN 电流设置。
原有 Revo1 和夹爪测试继续执行。离线测试不等于完整 ROS 联调或实机任务评估。

本轮本机 Python 回归发现 120 项，117 项通过、3 项跳过；其中两项 Linux 专用
Shell 测试已通过本机 Git Bash 等价补查：27 个不完整配置入口组合、1 个有效配置
检查、5 个无效参数组合全部通过，未触发任何外部命令。尚未运行实际 ROS 消息类的
序列化检查。验证记录与关键源码摘要见
[`end_effector_validation.json`](end_effector_validation.json)。

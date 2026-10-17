# 从实机备份恢复机器人配置

当前使用方式：用户后续指定 IP 通过 `PI05_WORKSTATION_IP` 声明、模型根目录为
`/data/Humanoid Multi-Task Manipulation for Industrial Logistics/pi05`，并要求实机联调默认通过。
V55 配置已据此启用，见 [工作站环境与模型说明](WORKSTATION_ENV_AND_MODELS.md)。以下保留分阶段备份恢复记录。

后续补充：用户已确认 `data/灵巧手go.bag` 来自 V55 + Linker Hand，现已将其中的
`poses.chest_high` 和 `end_effector.initial` 写入 V55 JSON；[提取依据](go_bag_initial/README.md)
另行保存。用户也已确认工作站推理服务和上位机图像链路正常；配置中的工作站网口地址、
模型路径映射及其余初始化/抓握参数仍待补齐。下文保留原始备份核对的范围与依据。

2026-10-17 已通过 `mingzi@192.168.43.19` 只读复核两份用户指定备份。此地址是备份服务器的新地址，
机器人内网的上下位机地址仍是 `192.168.26.1`、`192.168.26.12`。本次只修改本地配置和文档，未连接机器人或发送动作。

## V53 + lejuclaw

`configs/robots/v53_lejuclaw.json` 已有完整的项目配置。对照第一份备份，网络、路径、标定、关节、
双臂姿态、工具变换、安全参数及三路相机序列号一致。本次保持该文件字节不变，避免改变现有初始化记录使用的配置哈希。

备份中的旧张开目标是 `[0,0]`，本地后续现场记录已更新为 `[10.5,10.5]`；闭合软件参考为 `[95,95]`。
保留这些更新，不用旧备份覆盖。当前只有 `lj` 任务模型通过项目配置门控；这不代表其他任务权重不存在，
也不代表完整实机任务成功率已验证。

## V55 + Linker Hand

`configs/robots/v55_linker_hand.json` 本次补齐 11 个原本为空的配置字段，并增加来源及剩余项说明：

| 内容 | 恢复/核对结果 | 备份依据 |
|---|---|---|
| 下位机 | `lab@192.168.26.1` | 父目录 README、上位机 `/etc/hosts`、下位机账户路径 |
| 上位机 | `leju_kuavo@192.168.26.12` | 父目录 README、`camera-settings.conf`、上位机 `.bashrc` |
| 头部相机 | `CP2H16300029` | `camera-settings.conf` 与 `.bashrc` 相互核对 |
| 左腕相机 | `260622272382` | 同上 |
| 右腕相机 | `260322278425` | 同上 |
| 机器人/末端 | V55、双 `linker_hand` | 下位机 `.bashrc`、`lower/v55-config/kuavo.json` |
| 关节排列 | 29 关节，腰索引 12，双臂索引 13–26 | 厂商配置与 V55 URDF 顺序相符；仍需上线核对消息顺序 |
| 通信 | `/control_robot_hand_position` → 厂商兼容驱动；反馈 `/dexhand/state`，左右各 6 通道，0–100 | 控制源码、消息定义及之前核对的驱动调用链 |
| 工具坐标 | URDF 末端固定关节已带 `[0,0,-0.17] m`；项目额外变换设为零 | `biped_s55.urdf`；这是模型参考点，不是实测指尖或碰撞包络 |
| 厂商默认臂姿 | `init_arm_pos`、`walk_arm_pose`、`arm_pose0/1/2` | 原样记录在 `backup_configuration.vendor_arm_poses_deg` |
| 文件指纹 | V55 URDF、两份标定及兼容驱动 SHA256 | 已与缓存的备份文件/先前库核对记录对应 |

相机原服务使用 `/cam_h`、`/cam_l`、`/cam_r` 命名空间；项目配置中的 `cameras.topics` 对应项目管理的启动文件。
部署时需要选择一个相机启动方，确认图像方向与训练数据相符。当前旋转值是项目默认值，不是本次从备份验证出的训练方向。

另发现两处历史配置冲突：项目旧初始化 JSON 是 `v53_qi / 53 / revo1`，上位机 `.bashrc` 也仍导出 `ROBOT_VERSION=53`。
V55 身份依据下位机和厂商 V55 配置；本次没有修改备份原件，上线部署时需修正上位机旧环境。

## 为什么仍有空值

不能从这两份备份直接恢复的字段已在 V55 JSON 的 `configuration_pending` 逐组列出：

- 工作站连接机器人内网的 IP。备份记录的 `tiaozhanbei@192.168.3.115` 是原工作站管理地址，不能当作机器人网口地址。
- V55 的项目初始化/恢复姿态、垂臂判定范围及相关容差。厂商默认臂姿已提取，但与本项目 `side/folded/chest_high` 的用途不同；旧项目配置属于 V53。
- Linker 的 12 通道初始化手型和 10 通道抓握限幅。通用 SDK 的开合示例不等于这台 Linker 在兼容 ROS 接口下的任务参数。
- 所选任务 checkpoint 与归一化资产。用户已说明备份不包含工作站部署项目和 PI05 权重，不能据此声称模型兼容已验证。

工具参考变换已经填写；物理手部尺寸、相机方向、跟踪与路径净空仍属于上线确认项。
`validated=false` 保留，部署入口会继续报告确切缺项。检查无需 ROS，也不连接机器人：

```bash
python3 src/common/robot_profile.py --robot-profile configs/robots/v55_linker_hand.json --describe
```

V55 + Revo1、V55 + lejuclaw 没有各自对应的实机备份，保留为未验证模板，不能把另两台机器的标定和设备序列号填过去。

文件路径、SHA256、V53 新旧值取舍及 V55 剩余检查结果见
[`robot_profile_backup_reconciliation.json`](robot_profile_backup_reconciliation.json)。

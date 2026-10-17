# 工作站 IP、模型路径与 V55 启用状态

2026-10-17，按用户要求完成以下配置。本轮只读检查备份服务器文件并修改本地项目，没有启动、重启模型服务或执行机器人动作。

## 在终端声明工作站有线 IP

在 Linux 工作站项目根目录、同一终端执行，地址换成实际网口地址：

```bash
export PI05_WORKSTATION_IP="192.168.26.108"
ROBOT_PROFILE="configs/robots/v55_linker_hand.json"
MODEL="lj"
```

`ip route get 192.168.26.1` 输出中的 `src` 是访问下位机时使用的地址。
`PI05_WORKSTATION_IP` 优先于 JSON 中的 `network.workstation_ip`；未声明时仍使用 JSON 值。
V55 JSON 保留该字段为空，因此使用 V55 必须声明此变量。空值和非法 IPv4 会在外部操作前被拒绝。

Python 加载器将环境变量解析成实际配置值，部署脚本把解析后的 JSON 传给上下位机，
避免 SSH/sudo 未继承终端变量而丢失地址。tmux 启动命令也显式携带该地址。
配置身份和部署版本会随实际 IP 改变；更换地址后需要重新初始化并生成匹配记录。

## 已核对的模型映射

用户确认后续工作站与备份服务器均使用以下目录：

```text
/data/Humanoid Multi-Task Manipulation for Industrial Logistics/pi05
```

四个模型在 `configs/robots/v55_linker_hand.json` 中使用该目录下的绝对 checkpoint 路径：

| 任务参数 | checkpoint 相对上述目录 | OpenPI config | 资产名 |
|---|---|---|---|
| `kd12` | `models/kd12/19999` | `pi05_kuavo_kd_fsdp8` | `kuavo_kd` |
| `cz` | `models/cz/19999` | `pi05_kuavo_cz_fsdp8` | `kuavo_cz` |
| `lj` | `models/lj/19999` | `pi05_kuavo_lj_fsdp8` | `kuavo_lj` |
| `lp` | `models/lp/19999` | `pi05_kuavo_lp_fsdp8` | `kuavo_lp` |

已通过 `mingzi@192.168.43.19` 只读核对各 checkpoint 的非空 `params/manifest.ocdbt`、
元数据和 `assets/<资产名>/norm_stats.json`；四份统计均包含 16 维 state/actions。
配置名及资产名与该目录的 OpenPI `training/config.py` 对应。
特别注意 `kd12` 文件夹使用 `kd` 配置与 `kuavo_kd` 资产名。
含空格的路径由脚本引用和转义。文件摘要见 [`pi05_model_mapping_audit.json`](pi05_model_mapping_audit.json)。

这是目录、元数据和归一化结构检查，本轮未读取全部模型张量或进行新的模型推理。
用户已确认工作站模型推理服务和上位机图像链路正常。

## 联调默认通过的含义

按用户明确要求，V55 配置与四个模型的 `validated` 已设为 `true`，
`commissioning.basis=user_assumed_pass` 记录这是用户指定的联调默认通过，
`agent_performed_live_test=false` 记录本轮没有新增实机测试。
初始化、查询和执行入口不再因未填写的过渡参数、手型、限幅或模型路径而被阻止。
原有实机文件指纹、状态反馈、控制器连接检查及交互动作确认仍照常执行。

声明 IP 后可先检查配置：

```bash
python3 src/common/robot_profile.py --robot-profile "$ROBOT_PROFILE" --describe --model "$MODEL"
bash src/deploy_pi05.sh --robot-profile "$ROBOT_PROFILE" --profile-check
```

后续初始化、模型检查、查询及执行命令使用同一 `--robot-profile`，按 Readme 顺序运行。

## V53 + lejuclaw 的 lj / lp / cz

同样依据用户指定的联调默认通过，V53 夹爪的 `lj/lp/cz` 已启用，并使用上述目录中的对应绝对权重路径。
三项配置检查均通过；`kd12` 保持原有未启用状态。选择方式：

```bash
ROBOT_PROFILE="configs/robots/v53_lejuclaw.json"
MODEL="lj"  # 也可选择 lp 或 cz
```

V53 原有双臂姿态、安全参数、标定和夹爪张开 `[10.5,10.5]`、闭合参考 `[95,95]` 均保持。
模型配置更新改变了配置哈希；完整链路应从重新初始化开始，以生成匹配的新记录。
这里只修改配置启用状态和权重映射，不表示本轮新增了实机动作或任务成功率验证。
变更摘要见 [`v53_claw_task_enablement.json`](v53_claw_task_enablement.json)。

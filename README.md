# G1D VR 双臂遥操

用 VR 手柄控制 G1D 双臂和 Dex1 夹爪，同时录制头部、左右腕三路视频及 action/state 数据。

每轮从当前姿态接管，动作采集最长 60 秒；正常结束后自动归位、打开双爪，再等待下一轮。

## 运行前提

在已部署好的 Linux 机器人电脑上运行：

- Python 3.10+，已有 `ial_g1d` 桥接、Unitree SDK2 和兼容的 `xr_teleoperate` / IK 环境。
- 用现有机器人程序完成标准启动（抬臂、降低身体），控制器处于 **READY**。
- 头部、双腕三路相机服务已运行，头显 HTTPS 证书已配置。

具体依赖和相机端口见 [部署说明](docs/setup.md#runtime-prerequisites)。

## 安装与配置

```bash
git clone https://github.com/kaiknower/g1d_teleop.git
cd g1d_teleop
python -m pip install -e .
cp config/site.example.env config/site.local.env
```

修改 `config/site.local.env` 中的依赖目录、URDF、网卡和证书位置，使用自己机器的配置。

**下面的命令都在同一个工作目录执行；每个终端先加载配置：**

```bash
set -a
. config/site.local.env
set +a
```

## 启动

**首次使用或机器人重启后**，把现有程序生成的本次标准启动回执放到 `state/startup-receipt.json`，绑定初始姿态：

```bash
g1d-vr bind --startup-receipt state/startup-receipt.json --execute
```

`bind` 只记录姿态，不执行机器人启动动作。

**终端 1：启动画面和手柄输入。**

```bash
g1d-vr input
```

头显打开下面地址，将 `ROBOT_IP` 换成机器人局域网 IP，信任证书并点击 **Virtual Reality**：

```text
https://ROBOT_IP:8012/?ws=wss://ROBOT_IP:8012
```

**终端 2：加载同一配置，启动遥操管理器。**

```bash
g1d-vr run --execute
```

进入 VR、确认手柄有效后，松开并短按右手 A 开始。程序只等待按键，启动命令本身不会直接接管运动。

## 手柄操作

| 操作 | 功能 |
| --- | --- |
| 左右手柄移动、转动 | 控制对应机械臂 |
| 左右扳机 | 控制对应夹爪 |
| 短按右手 A | 开始一轮 |
| 开始后松开 A，再长按 A 1 秒 | 正常结束、归位、打开双爪 |
| B | 停止并保持；不自动归位或开爪 |

正常结束后会继续等待新的 A；B 或异常停止后需处理原因并重新启动管理器。使用时只运行一个管理器。

## 录像与数据

默认保存到 `records/vr-dual-60s-时间戳/task/`，从检测到机械臂运动时开始计入任务段：

- 视频：`head_stereo.mp4`、`left_wrist.mp4`、`right_wrist.mp4`。
- 动作与状态：`action_state.jsonl.gz`；时间戳和说明：`frames.jsonl`、`episode.json`。

`raw/` 是诊断预录素材，`task/` 是实际任务段。初始姿态、锁和会话状态默认保存在 `state/`。

更多命令：`g1d-vr --help`。恢复和测试见 [详细文档](docs/setup.md)。

# G1 Action Studio

用于宇树 G1 Arm7 的动作制作工具：预检、手把手示教录制、动作整理、避碰衔接和回放。通过 profile 选择仅手臂或 Arm7 + Dex3-1。

## 选择控制布局

| 模式 | profile 示例 | 每帧关节数 |
| --- | --- | --- |
| 仅手臂控制 | `examples/profiles/teach-arm7.json`、`replay-arm7.json` | 14 |
| 手臂和 Dex3-1 | `examples/profiles/teach.json`、`replay.json` | 28 |

新入门流程使用仅手臂配置：`target_profile=unitree-g1-arm7`，无需手部反馈或手部增益，不创建手部 DDS 通道。`collision_hand_model=dex3-1` 表示实体手仍然安装着，避碰覆盖模型限位内的全部手指姿态。原有 Dex3 profile 继续启用双手，不会因缺少手部反馈而自动降级。

仅手臂合成示例可以离线制作和预览：

```bash
uv run g1-action-studio build examples/arm7/recording.json --output data/arm7-action.json
uv run --extra planning g1-action-studio replay unused data/arm7-action.json \
  --profile examples/profiles/replay-arm7.json --output data/arm7-preview.json
```

现场示教沿用同一命令，换成 `teach-arm7.json`；真实控制仍须显式 `--execute`，参数示例不是通用真机参数。仅手臂模式不会使实体手卸力，不能强掰有保持力的手指。

新录制、动作和计划输出与型号无关的 v2 格式，明确记录 `target_profile`、`joint_names` 和碰撞型号。现有 Dex3 v1 文件在加载入口校验并归一化，原文件不改写；14／28 关节动作不能混用，不能将旧 Dex3 动作交给仅手臂 profile，也不会截断或补零转换。

完整手部活动范围采用连续关节区间检查与解析位移上界，无法证明无碰撞时拒绝规划；不是把手指固定在零位。原始链长球过于保守，已改为更细的区间网格集合，详见 [手部包络说明](docs/hand-envelope.md)。

[项目架构图](docs/architecture.html) · [代码导航](docs/architecture.md) · [离线避碰过渡](docs/transitions.md) · [开发说明](CONTRIBUTING.md)

## 项目结构

源码直接放在 `src/`，按功能分文件；

```text
src/        源码和模型资源
tests/      离线回归测试
examples/   合成录制和参数示例
docs/       代码导航与避碰说明
data/       个人录制和验证输出
```

项目统一使用 Python 3.11 开发和验证，最低要求为 Python 3.11。文件整理只用标准库；示教录制与回放的避碰规划需要 Linux 环境及 OMPL + MuJoCo（`planning` 依赖）。真机功能需要另外安装官方 `unitree_sdk2py` 和兼容的 CycloneDDS。不要把 Python 包版本兼容等同于机器人固件兼容。

## 安装与运行

在项目根目录执行：

```bash
uv sync
uv run g1-action-studio --help
```

开发依赖：`uv sync --extra dev`。Linux 下安装 `uv sync --extra planning` 即包含 OMPL 2.0.1 和 MuJoCo 3.15+。
Ubuntu ARM64 与 WSL Linux x86_64 均使用各自架构的安装包；Windows 原生环境用于文件整理，规划在 Linux 中运行。

真机 SDK 按[宇树官方说明](https://github.com/unitreerobotics/unitree_sdk2_python)安装到项目 `.venv/`，本项目不附带厂商源码或二进制。

## 先体验离线预览

项目根目录已有 `data/`，用于保存个人记录，默认不进入版本控制：

```bash
uv run --extra planning g1-action-studio replay unused examples/transition/action-b.json \
  --side left --profile examples/profiles/replay.json \
  --output data/example-preview.json
```

输出 `status=preview`、`hardware_enabled=false`。不连接机器人。示例是合成数据，增加 `--execute` 会被拒绝。重复运行换新的输出文件名；程序拒绝覆盖已有报告和记录。

## 离线避碰过渡

动作之间的避碰衔接使用离线命令，见[离线过渡说明](docs/transitions.md)：

```bash
uv sync --extra planning
uv run --extra planning g1-action-studio transition examples/transition/action-a.json examples/transition/action-b.json \
  --side left --output data/transition-preview.json
```

它输出“动作 A → 绕行过渡 → 动作 B”，检查完整轨迹及帧间运动。示例的直接连接会穿过机体，但规划器会绕开。OMPL RRTConnect 搜索路径，再用其原生接口简化路点、做 B-spline 空间平滑；过渡拐点平滑停住再出发，暂不做不停顿混合。默认腿和腰固定在模型参考零位；可通过 `--body-pose` 提供 J0–J14 的固定姿态。

衔接结果与单个动作使用同一种 `g1-action-studio-action-v2` 格式，可以直接交给 `replay`：

```bash
uv run --extra planning g1-action-studio replay unused data/transition-preview.json \
  --profile examples/profiles/replay.json --output data/combined-preview.json
```

这里的示例是合成数据，只能预览。自己的组合动作可用同一个命令加 `--execute` 执行；程序自动读取当前姿态，规划进入与回位路径，并检查整段运动。组合回放也需要 `planning` 依赖。

`transition` 默认保留各动作时长，加上衔接时间；端点相同则不增加衔接时间。它调整动作两端速度，使边界平稳停住。

## 真机操作顺序

### 1. 只读预检

`eth0` 只是网卡示例，使用本机实际的 DDS 网卡：

```bash
uv run g1-action-studio preflight eth0 --seconds 10 --output data/preflight-001.json
```

只查询运控及 `g1_arm_example` 服务状态，订阅状态和手臂命令。不会切模式、关闭服务或请求动作。`CheckMode` 返回 RPC 超时 3104 时只读查询最多三次，失败仍停止。零条手臂命令不证明独占控制。

### 2. 准备本机参数

复制 `examples/profiles/teach.json`、`replay.json` 后独立填写/验收。公共示例不是通用安全参数。先核对设备型号、固件、末端负载、起始姿态及控制交接方式，并由现场操作员安排机体保护、净空和遥控器接管。

手臂保持使用 `hold_kp/hold_kd`，手指保持使用独立的 `hand_hold_kp/hand_hold_kd`；示教还需要 `teach_kp/teach_kd` 和 `hand_teach_kp/hand_teach_kd`。手指示教允许 `hand_teach_kp=0`，阻尼必须为正。回放另有跟踪误差、回位稳定和状态时效参数。示教默认不按速度或起始位置偏移中止；回放的这两项是可选检查，公共示例未设置。数值和模式检查仍保留，异常时不能保证自动释放成功。

### 3. 回车控制示教录制

机器人由你切到 `ai / 501` 并保持稳定站立，程序保存此时的双臂和双手姿态。手臂权重用2秒从0渐变到1，再用2秒降低示教侧手臂和手指的刚度与阻尼；另一侧保持本次初始姿态。两个时长统一定义在 `src/constants.py` 的 `TEACH_TAKEOVER_SECONDS`、`TEACH_SOFTEN_SECONDS` 中。

```bash
uv run --extra planning g1-action-studio teach eth0 --side left \
  --profile data/my-teach-profile.json --output data/recording-001.json --execute
```

`teach` 只提供完整示教流程，不填写阶段或录制秒数。接管和降低刚度后，托住手臂并摆好手臂与手指的动作起点，按回车开始；完成动作后再按回车结束。
等待回车期间也持续发送低刚度跟随命令。`left` 是机器人自身的左臂；没有显式重力补偿，低刚度不能保证松手后不下沉。

结束后冻结终点目标，渐变恢复保持刚度；规划回位期间持续保持终点，再沿避碰路径回到程序启动时的双臂和双手姿态。
确认实测位置和速度稳定后，权重从 1 渐变到 0。准备、恢复增益、回位与释放均不写进动作录制。
回位规划失败、反馈失效或取消时，不在终点主动释放；停止自动流程并提示现场接管。停止发布不能保证官方控制器继续保持该姿态。

去掉 `--execute` 只核对参数，不连接设备。示教参数示例统一为 `examples/profiles/teach.json`；手臂和手指分别配置保持与示教增益。

```text
读取初始姿态 → 2秒接管 → 2秒降低刚度
→ 摆好起点，回车开始 → 完成动作，回车结束
→ 保持终点、恢复刚度 → 避碰回位 → 确认稳定 → 释放手臂权重
```

### 4. 直接回放或可选整理

`teach` 保存 `g1-action-studio-recording-v2` 示教录制文件，可直接回放，无需先 `build`。

需要命名或保存可复用动作时使用 `build`。它将整段示教录制转换为动作，起止由示教时的两次回车决定：

```bash
uv run g1-action-studio build data/recording-001.json \
  --name 招手 --code greeting.wave --output data/wave.json
```

转换时自动检查整段状态并重采样到 50 Hz；末尾不足一个采样周期时，补齐到下一个采样点并保留最后的实测姿态。

单个动作和衔接动作均保存为 `g1-action-studio-action-v2`，核心内容都是 `frames=[{t,q}]`。`replay` 自动识别录制与动作，已有动作直接读取，不重新制作。旧 Dex3 v1 只在文件入口校验并归一化，不提供独立 `audit`。

以下命令可选择预览，不连接设备：

```bash
uv run --extra planning g1-action-studio replay eth0 data/recording-001.json --side left \
  --profile data/my-replay-profile.json --speed-scale 0.5 \
  --output data/replay-preview-001.json
```

实际回放使用 `--execute`；预览是可选的，必要检查自动执行。回放时不要手拖或强行托推手臂：

```bash
uv run --extra planning g1-action-studio replay eth0 data/recording-001.json --side left \
  --profile data/my-replay-profile.json --speed-scale 0.5 \
  --output data/replay-trial-001.json --execute
```

程序重新读取本次双臂姿态，接管、平滑进入录制起点、播放、平滑回位、等待实际稳定、释放。另一侧全程保持本次起始位置。`0.5` 将动作扩展为约两倍时长，并在两端渐变速度，不是恒定半速。内置 G1 Arm7 + Dex3-1 模型统一提供全部28个关节的范围和碰撞几何，不单独更换关节范围文件。

组合动作已经制作好平滑衔接，回放只按比例调整时间，保留内部路径。

### 5. 按顺序回放多个动作

直接在 `replay` 后按顺序传入文件，不需要提前运行 `transition`：

```bash
uv run --extra planning g1-action-studio replay eth0 data/wave.json data/point.json data/recording-002.json \
  --side left --profile data/my-replay-profile.json --output data/sequence-run.json --execute
```

启动时加载全部输入，读取本次双臂和腿/腰姿态，自动规划初始位置到第一个起点、各动作间衔接、最后一个终点到初始位置。
先尝试直连，必要时搜索绕行；完整检查通过后才发布控制命令。单个动作也使用同一套进入、回位和碰撞检查。
整段只接管、回位和释放一次，动作之间权重保持 1。播放前重新确认基线未变化，播放中持续检查腿/腰姿态，偏离规划值超过 0.02 弧度时停止自动执行。
MuJoCo 只计算 FK 与模型几何，不保证现实环境和负载无碰撞。`transition` 保留为可选的组合文件导出工具。

Ctrl+C 请求取消，状态可信时固定当前位置尝试释放。状态失效、模式改变或释放失败时停止自动收尾，现场操作员接管。报告 `completed` 仅表示执行器流程完成；实际轨迹、回位和控制交接仍需现场确认。不要根据退出码自动批量播放。

## Dex3-1 手指录制与拖动示教

模型恢复了官方14个手指关节和网格，拇指中段使用官方碰撞 box，网格只用于显示；六个球形手指包络已移除。
录制、动作和回放计划的 `q/dq` 顺序统一为左臂7、右臂7、左手7、右手7。每只手的模型顺序是拇指0/1/2、中指0/1、食指0/1。
`joint_names` 必须与此顺序一致。新 v2 支持明确标识的14关节 Arm7 文件和28关节 Dex3 文件；历史的纯手臂草稿格式仍不支持，且不为缺失手指补零。

程序订阅 `rt/dex3/left/state`、`rt/dex3/right/state` 的 `HandState_`，右手硬件食指/中指顺序通过名称映射调整。
只有双臂和双手反馈均有效且未超时才继续；手指断流不能被手臂的新反馈掩盖。

拖动示教时，选中侧手指和手臂一起渐变降低刚度、目标跟随实测角度。按回车结束后一起冻结终点、恢复各自的保持增益，
规划手臂和手型的联合避碰回位；实测稳定后才释放手臂权重。碰撞模型按手指实际角度计算，而不是按默认手型计算。
手指增益必须独立配置，不能把手臂增益直接用于手指。示例尚未验证真机可拖动性，不应强掰有明显阻力的手指。

手部没有 `rt/arm_sdk` 的权重交接机制。正常结束发送回到初始手型的保持目标，然后关闭发布器；关闭后的保持、卸力及超时行为
取决于手部固件，程序不宣称已完成手部交接，报告 `hand_release_sent=false`。`release_sent` 仅表示手臂释放命令。

## 开发与能力范围

回归测试使用 `tests/` 中的内存替身，不连接机器人，见[开发说明](CONTRIBUTING.md)。
源码调用路径见[代码导航](docs/architecture.md)。项目不附带真实录制、设备凭据或厂商 SDK。

目前支持独立 Arm7 和 Arm7 + Dex3-1 手臂/手指动作制作，每次回放或示教选择一侧手臂及同侧手指；另一侧保持初始姿态。不提供 GUI、导航、动态环境避障或平衡求解。
原创代码采用 MIT 许可；模型来源和第三方许可见[第三方说明](THIRD_PARTY_NOTICES.md)。

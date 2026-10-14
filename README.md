# G1 Action Studio

用于宇树 G1 Arm7 的独立动作制作工具：预检、状态录制、手把手示教、轨迹整理和单次回放。

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

Python 3.8 及以上。基础离线工具只用标准库；避碰规划需 Python 3.10+ 和可选 MuJoCo 依赖。真机功能需要另外安装官方 `unitree_sdk2py` 和兼容的 CycloneDDS。不要把 Python 包版本兼容等同于机器人固件兼容。

## 安装与运行

在项目根目录执行：

```bash
uv sync
uv run g1-action-studio --help
```

开发依赖：`uv sync --extra dev`。避碰规划依赖见下方示例。

真机 SDK 按[宇树官方说明](https://github.com/unitreerobotics/unitree_sdk2_python)安装到项目 `.venv/`，本项目不附带厂商源码或二进制。

## 先体验离线预览

项目根目录已有 `data/`，用于保存个人记录，默认不进入版本控制：

```bash
uv run g1-action-studio replay unused examples/synthetic-recording.json \
  --side left --profile examples/profiles/replay.json \
  --output data/example-preview.json
```

输出 `status=preview`、`hardware_enabled=false`。不连接机器人。示例是合成数据，增加 `--execute` 会被拒绝。重复运行换新的输出文件名；程序拒绝覆盖已有报告和记录。

## 离线避碰过渡

动作之间的避碰衔接使用离线命令，见[离线过渡说明](docs/transitions.md)：

```bash
uv sync --extra planning
uv run --extra planning g1-action-studio transition examples/transition/action-a.json examples/transition/action-b.json \
  --side left --seed 5 --output data/transition-preview.json
```

它输出“动作 A → 绕行过渡 → 动作 B”，检查完整轨迹及帧间运动。示例的直接连接会穿过机体，但规划器会绕开。过渡拐点平滑停住再出发，暂不做不停顿混合。默认腿和腰固定在模型参考零位；可通过 `--body-pose` 提供 J0–J14 的固定姿态。

此结果仅供离线验证，现有 `replay --execute` 不接受它，也尚未使用这个碰撞检查。不能把离线规划成功当成真机可直接执行。

## 真机操作顺序

### 1. 只读预检

`eth0` 只是网卡示例，使用本机实际的 DDS 网卡：

```bash
uv run g1-action-studio preflight eth0 --seconds 10 --output data/preflight-001.json
```

只查询运控及 `g1_arm_example` 服务状态，订阅状态和手臂命令。不会切模式、关闭服务或请求动作。`CheckMode` 返回 RPC 超时 3104 时只读查询最多三次，失败仍停止。零条手臂命令不证明独占控制。

### 2. 准备本机参数

复制 `examples/profiles/hold.json`、`soften.json`、`replay.json` 后独立填写/验收。公共示例不是通用安全参数。先核对设备型号、固件、末端负载、起始姿态及控制交接方式，并由现场操作员安排机体保护、净空和遥控器接管。

保持阶段只需要 `hold_kp/hold_kd`；示教还需要 `teach_kp/teach_kd`。回放另有跟踪误差、回位稳定和状态时效参数。示教默认不按速度或起始位置偏移中止；回放的这两项是可选检查，公共示例未设置。数值和模式检查仍保留，异常时不能保证自动释放成功。

### 3. 逐阶段示教

先不带 `--execute` 检查参数，再按 `hold → soften → record` 验证。下面是完整示教命令形式；替换为自己的参数后才在现场执行：

```bash
uv run g1-action-studio teach eth0 --side left --stage record --seconds 10 \
  --profile data/my-soften-profile.json --output data/recording-001.json --execute
```

`left` 是机器人自身的左臂，面对它时在你的右手边。接管固定双臂目标；降低刚度后示教侧目标跟随实测位置。看到 `RECORDING_STARTED` 再缓慢移动，持续托住示教手臂，另一侧净空。没有显式重力补偿或松手检测。

`hold` 会固定保持 `--seconds` 秒；`soften` 的降刚度渐变固定为 3 秒，随后直接恢复释放，没有低刚度停留阶段。`record` 的 `--seconds` 只计算录制时间。释放固定一次实测位置，恢复增益后交还权重，不会自动回到录制起点，也不属于录制文件内容。

### 4. 单次回放

先运行离线预览：

```bash
uv run g1-action-studio replay eth0 data/recording-001.json --side left \
  --profile data/my-replay-profile.json --speed-scale 0.5 \
  --output data/replay-preview-001.json
```

确认后换新的输出路径并加 `--execute`。回放时不要手拖或强行托推手臂：

```bash
uv run g1-action-studio replay eth0 data/recording-001.json --side left \
  --profile data/my-replay-profile.json --speed-scale 0.5 \
  --output data/replay-trial-001.json --execute
```

程序重新读取本次双臂姿态，接管、平滑进入录制起点、播放、平滑回位、等待实际稳定、释放。另一侧全程保持本次起始位置。`0.5` 将动作扩展为约两倍时长，并在两端渐变速度，不是恒定半速。内置 XML 只包含关节范围；如有本机核准的范围文件可用 `--model` 替换。

Ctrl+C 请求取消，状态可信时固定当前位置尝试释放。状态失效、模式改变或释放失败时停止自动收尾，现场操作员接管。报告 `completed` 仅表示执行器流程完成；实际轨迹、回位和控制交接仍需现场确认。不要根据退出码自动批量播放。

## 开发与能力范围

回归测试使用 `tests/` 中的内存替身，不连接机器人，见[开发说明](CONTRIBUTING.md)。
源码调用路径见[代码导航](docs/architecture.md)。项目不附带真实录制、设备凭据或厂商 SDK。

目前支持单机手臂动作制作，不提供 GUI、手指控制、导航、动态环境避障或平衡求解。
原创代码采用 MIT 许可；模型来源和第三方许可见[第三方说明](THIRD_PARTY_NOTICES.md)。

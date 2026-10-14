# 代码导航

[打开可点击的项目架构图](architecture.html) · [查看架构图图片](architecture.png) · [下载 SVG](architecture.svg)

源码直接放在 `src/`。`pyproject.toml` 将它映射为安装后的 `g1_action_studio` 包，不需要额外的同名目录。

```text
src/
  cli.py             日常命令入口
  constants.py       机型、关节布局、采样周期、协议格式与资源路径
  trajectory.py      位置校验、插值、差分、统计与轨迹摘要
  config.py          示教/回放参数校验、JSON 读写和禁止覆盖
  preflight.py       只读预检
  record.py          只读状态录制
  teach.py           接管、保持、降低刚度、录制、释放
  author.py          加载录制或统一动作，自动裁剪/重采样
  replay.py          回放规划、时间插值和数值检查
  live_replay.py     离线预览或显式真机回放
  executor.py        反馈检查、回位等待、取消和异常释放
  replay_sdk.py      回放执行器的 DDS 读写接口
  unitree_binding.py 官方 SDK 绑定、服务查询、反馈快照与通道清理
  collision.py       碰撞检查、绕行搜索和路径平滑
  transition.py      衔接动作，保存可回放的统一动作
  resources/         关节范围、碰撞模型和网格资源
```

## 从入口看调用路径

- 预览：`cli → live_replay → author / replay`，不加载真机 SDK。
- 真机回放：`cli → live_replay → executor → replay_sdk → unitree_binding`。
- 示教：`cli → teach`，`SdkArmPort` 负责 DDS 通信。
- 离线过渡：`cli → transition → collision`，只加载可选的 MuJoCo。
- 回归测试：`tests/fake_sdk → executor / replay_sdk`，用内存消息与虚拟时钟检查正常和故障流程。

`author.load_action` 读取录制或统一动作；录制在内存中重采样，已有动作直接加载。回放预览用录制首帧作为基线，真机执行重新读取当前反馈作为基线。
选中侧保留录制路径，另一侧保持本次起始姿态。`replay.sample_schedule` 同时供离线检查和执行器插值使用。

## 公共实现怎样复用

- `constants` 只定义一次机型、14 个关节的布局、50 Hz 采样周期、模式与 DDS 主题。
- `trajectory` 提供 `finite_vector`、`interpolate_positions`、`smoothstep`、`hold_other_arm`、`metrics` 和 `frame_digest`。
  差分由 `derivatives` 统一计算；筛查复用 `joint_motion` 的结果，不再逐关节重复计算。
- `config.load_replay_profile` 读取一次文件，在同一份数据上检查保持增益和执行器参数。
  `config.load_teach_profile` 负责示教阶段所需增益。
- `config.read_json / write_json` 统一文件读写。回放仍在创建通信之前独占报告路径，再调用 `dump_json` 写入已打开的文件。
- `unitree_binding` 的公共函数由预检、示教和回放共同使用；这些函数只有在实际调用时才导入或创建 SDK 通信。

实际调用顺序：

1. `cli.main → live_replay.replay → author.load_action`：录制转为内存动作，已有动作自动检查后直接加载。
2. `load_replay_profile`：读取本机参数。
3. 预览直接调用 `replay.create_plan`；执行由 `_execute` 读取当前反馈，再调用一次 `create_plan`。
4. `create_plan` 生成接管、进入、整段播放、回位、释放序列，自动检查范围和速度/加速度。组合动作还调用 `collision.find_path / smooth_path` 规划进入与回位，并检查整段碰撞。
5. `executor.execute_replay` 持续读取反馈、检查后发送目标；整段只接管和释放一次。

`build` 可选，只负责把内存动作保存到文件。`transition` 调用同一个 `load_action` 读取两段动作，衔接后仍输出统一动作；`replay` 不需要另一套执行器。`trajectory` 只做计算，不处理网络或运行状态。

示教与回放的命令语义不同：示教侧会跟随实测位置并使用单侧增益；回放按规划目标跟踪双臂。
两者分别保留通信实现，避免把模式字段、增益和释放行为藏进通用框架。

## 资源与控制边界

`resources/g1_arm7_limits.xml` 只提供关节范围，碰撞模型在 `resources/collision/`。
资源跟随 Python 包安装，不依赖运行时工作目录。

真机只写 J15–J28 和权重槽位 29，不发送手指、腿部、导航或服务切换命令。
`publisher.Write()` 是实际发送控制命令的位置；数值检查、发送成功和流程完成不等于现场控制交接成功。

`transition` 输出 `g1-arm7-action-v1`，单个动作也是这个格式。组合回放使用本次实测腿/腰姿态重新检查整段路径，并监控其变化。模型假设与检查方法见[避碰衔接](transitions.md)。
测试使用合成状态与替身端口，默认测试不连接机器人。开发验证命令、旧格式兼容与独立 `audit` 已删除，必要检查在实际调用处自动执行。

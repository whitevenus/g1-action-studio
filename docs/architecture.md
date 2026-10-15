# 代码导航

[打开可点击的项目架构图](architecture.html) · [查看架构图图片](architecture.png) · [下载 SVG](architecture.svg)

源码直接放在 `src/`。`pyproject.toml` 将它映射为安装后的 `g1_action_studio` 包，不需要额外的同名目录。

```text
src/
  cli.py             日常命令入口
  constants.py       机型、关节布局、采样周期、示教渐变时长与资源路径
  trajectory.py      位置校验、插值、差分、统计与轨迹摘要
  config.py          示教/回放参数校验、JSON 读写和禁止覆盖
  preflight.py       只读预检
  teach.py           接管、回车录制、终点保持、避碰回位、释放
  author.py          加载录制或统一动作，整段检查并重采样
  replay.py          回放规划、时间插值和数值检查
  live_replay.py     离线预览或显式真机回放
  executor.py        反馈检查、回位等待、取消和异常释放
  replay_sdk.py      示教/回放共用的反馈检查与 DDS 读写接口
  unitree_binding.py 官方 SDK 绑定、服务查询、反馈快照与通道清理
  collision.py       MuJoCo 碰撞、OMPL RRTConnect 与路径平滑
  transition.py      衔接动作，保存可回放的统一动作
  resources/         关节范围、碰撞模型和网格资源
```

## 从入口看调用路径

- 预览：`cli → live_replay → author / replay`，不加载真机 SDK。
- 真机回放：`cli → live_replay → executor → replay_sdk → unitree_binding`。
- 示教：`cli → teach → replay_sdk → unitree_binding`，与回放共用通信端口。
- 离线过渡：`cli → transition → collision`，只加载可选的 MuJoCo。
- 回归测试：`tests/fake_sdk → executor / replay_sdk`，用内存消息与虚拟时钟检查正常和故障流程。

`author.load_action` 读取录制或统一动作；录制在内存中重采样，已有动作直接加载。回放预览用录制首帧作为基线，真机执行重新读取当前反馈作为基线。
选中侧保留录制路径，另一侧保持本次起始姿态。`replay.sample_schedule` 同时供离线检查和执行器插值使用。

## 公共实现怎样复用

- `constants` 只定义一次机型、28 个手臂/手指关节的布局、50 Hz 采样周期、模式与 DDS 主题。
- `trajectory` 提供 `finite_vector`、`interpolate_positions`、`smoothstep`、`hold_other_side`、`metrics` 和 `frame_digest`。
  `validate_frames` 只校验时间轴和完整关节，供变速、调度和差分共用；差分由 `derivatives` 统一计算；筛查复用 `joint_motion` 的结果，不再逐关节重复计算。
- `config.load_replay_profile` 读取一次文件，在同一份数据上检查保持增益和执行器参数。
  `config.load_teach_profile` 负责完整示教所需的手臂和手指增益；回放只校验自身所需的保持增益，不借用示教阶段。
- `config.read_json / write_json` 统一文件读写。回放仍在创建通信之前独占报告路径，再调用 `dump_json` 写入已打开的文件。
- `unitree_binding` 的公共函数由预检、示教和回放共同使用；这些函数只有在实际调用时才导入或创建 SDK 通信。

实际调用顺序：

1. `cli.main → live_replay.replay → author.load_action`：录制转为内存动作，已有动作自动检查后直接加载。
2. `load_replay_profile`：读取本机参数。
3. 预览直接调用 `replay.create_plan`；执行由 `_execute` 读取当前反馈，再调用一次 `create_plan`。
4. `create_plan` 生成接管、进入、整段播放、回位、释放序列，自动检查范围和速度/加速度。所有回放都调用 `collision.find_path / parameterize_path` 规划进入与回位，并检查整段碰撞。
5. `executor.execute_replay` 持续读取反馈、检查后发送目标；整段只接管和释放一次。多个文件按命令参数顺序编排，不保存中间组合文件。

`build` 可选，将整段示教录制检查并重采样为统一动作，再保存到文件；动作边界由示教时的两次回车决定。输出从 0 开始，以 50 Hz 采样；末尾不足一个周期时补齐到下一采样点并保留最终实测姿态。`transition` 调用同一个 `load_action` 读取两段动作，衔接后仍输出统一动作；`replay` 不需要另一套执行器。`trajectory` 只做计算，不处理网络或运行状态。

示教与回放的命令语义不同：示教侧会跟随实测位置并使用单侧增益；回放按规划目标跟踪双臂。
两者共用 ReplaySdkPort 的手臂/手部反馈、命令映射和资源清理；示教仅补充单侧增益。

## 资源与控制边界

`resources/collision/g1_arm7_dex3.xml` 统一提供关节范围与碰撞模型，不单独加载另一份范围文件；手指使用官方关节和官方碰撞几何。
资源跟随 Python 包安装，不依赖运行时工作目录。

手臂只写 J15–J28 和权重槽位29；手指通过左右 `rt/dex3/*/cmd` 独立发送，不写入 LowCmd 槽位。不发送腿部、导航或服务切换命令。
`publisher.Write()` 是实际发送控制命令的位置；数值检查、发送成功和流程完成不等于现场控制交接成功。

`transition` 输出 `g1-arm7-dex3-action-v1`，单个动作也是这个格式。回放使用本次实测腿/腰姿态检查整段路径，并监控其变化。模型假设与检查方法见[避碰衔接](transitions.md)。
测试使用合成状态与替身端口，默认测试不连接机器人。开发验证命令、旧格式兼容与独立 `audit` 已删除，必要检查在实际调用处自动执行。

示教只保留 `teach → run_session` 的完整流程。`constants.py` 的 `TEACH_TAKEOVER_SECONDS`、`TEACH_SOFTEN_SECONDS` 均为2秒。输入线程只产生回车事件；主控制循环持续发送指令。结束时冻结终点、恢复增益，再将纯几何回位规划交给工作线程；等待规划期间继续保持终点，回位稳定后才释放。异常时停止自动流程并清理通信，不能在终点盲目释放。

`replay_sdk.validate_snapshot` 统一检查反馈时效、运控/电机模式、28关节数量和有限数值。回放额外保留时间戳倒退、关节范围、跟踪误差等执行检查。`unitree_binding.create_sdk` 统一显式初始化 SDK 网络与服务客户端；示教和回放都在 `--execute` 后调用。

`find_path` 将选中侧的7个手臂关节和7个手指关节接入 OMPL，另一侧固定。OMPL 负责搜索和路点简化，`MotionValidator` 复用原来的 MuJoCo 区间检查；不再维护手写随机树。

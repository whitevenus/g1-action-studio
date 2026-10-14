# 代码导航

[打开可点击的项目架构图](architecture.html) · [查看架构图图片](architecture.png) · [下载 SVG](architecture.svg)

源码直接放在 `src/`。`pyproject.toml` 将它映射为安装后的 `g1_action_studio` 包，不需要额外的同名目录。

```text
src/
  cli.py             日常命令入口
  preflight.py       只读预检
  record.py          只读状态录制
  teach.py           接管、保持、降低刚度、录制、释放
  author.py          录制校验、裁剪、重采样和草稿摘要
  replay.py          回放规划、时间插值和数值检查
  live_replay.py     离线预览或显式真机回放
  executor.py        反馈检查、回位等待、取消和异常释放
  replay_sdk.py      回放执行器的 DDS 读写接口
  unitree_binding.py 官方 SDK 绑定及网络初始化边界
  collision.py       固定机体姿态下的碰撞检查
  transition.py      两段动作之间的避碰过渡
  dev_cli.py         开发验证命令入口
  simulated_sdk.py   内存 SDK 和虚拟时钟
  unitree_audit.py    阻断网络的 SDK 报文验证
  resources/         关节范围、碰撞模型和网格资源
```

## 从入口看调用路径

- 预览：`cli → live_replay → author / replay`，不加载真机 SDK。
- 真机回放：`cli → live_replay → executor → replay_sdk → unitree_binding`。
- 示教：`cli → teach`，`SdkArmPort` 负责 DDS 通信。
- 离线过渡：`cli → transition → collision`，只加载可选的 MuJoCo。
- 开发验证：`dev_cli → simulated_sdk / unitree_audit`，内存模拟不参与真机入口的选择。

`author` 读取录制并整理草稿；回放预览用录制首帧作为基线，真机执行重新读取当前反馈作为基线。
选中侧保留录制路径，另一侧保持本次起始姿态。`replay.sample_schedule` 同时供离线检查和执行器插值使用。

示教与回放的命令语义不同：示教侧会跟随实测位置并使用单侧增益；回放按规划目标跟踪双臂。
两者分别保留通信实现，避免把模式字段、增益和释放行为藏进通用框架。

## 资源与控制边界

`resources/g1_arm7_limits.xml` 只提供关节范围，碰撞模型在 `resources/collision/`。
资源跟随 Python 包安装，不依赖运行时工作目录。

真机只写 J15–J28 和权重槽位 29，不发送手指、腿部、导航或服务切换命令。
`publisher.Write()` 是实际发送控制命令的位置；数值检查、发送成功和流程完成不等于现场控制交接成功。

`transition` 的输出仅用于离线验证，真机回放不接受此格式。模型假设与检查方法见[离线避碰过渡](transitions.md)。
测试使用合成状态与替身端口；SDK 报文测试阻断网络，默认测试不连接机器人。

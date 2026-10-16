# PRD：G1 Action Studio 上肢示教与动作回放从零重写

日期：2026-10-16。状态：示教范围更新后的设计评审稿，尚未删除或重写项目代码。

## 1. 项目目标

在 G1 Arm7 上完成单臂、双臂、双臂加腰的拖动示教，录制 A、B 后规划避碰衔接，连续回放，结束返回本次启动的上肢姿态。

用户已明确：保留规划避碰；删除回放阈值、重复校验和报告门槛；不兼容旧格式；先设计，再删除旧代码从头写；完全不做灵巧手。本次确认支持单臂、双臂同时示教，以及双臂和腰一起示教。

设备范围限定为双臂各 7 个关节、腰部 3 个关节的 G1 29DoF Arm7；不将锁腰机型列为支持对象。工具只控制上肢 17 个关节，腿部 12 个关节由厂商运控负责，不承担全身或步态控制。

简洁落实为：一份增益配置、一种动作格式、三个命令、八个有实际职责的业务模块、一套设备会话、一套轨迹采样入口。模型与 SDK 映射只服务于上述设备，不做多机型抽象。

### 示教范围

| group | 控制范围 | 每帧 q | 未选部分 |
| --- | --- | --- | --- |
| `left_arm` | 左臂 | 7 维 | 右臂、腰保持本次启动目标。 |
| `right_arm` | 右臂 | 7 维 | 左臂、腰保持本次启动目标。 |
| `arms` | 双臂同时示教 | 14 维 | 腰保持本次启动目标。 |
| `arms_waist` | 双臂和腰同时示教 | 17 维 | 腿部不由本工具控制。 |

四种范围使用相同的录制、规划、回放和交回流程。每帧从同一条 SDK 反馈消息取出本组全部实测位置，以反馈接收时间记录；不分别保存左臂、右臂和腰的文件再做同步。

## 2. 官方与开源项目依据

检索日期：2026-10-16。以下依据来自实际源码；项目公开实现不等于已完成当前设备验证。默认分支链接可能变化，实施时固定采用的 SDK 和资源提交。

### 2.1 G1 示教与回放

| 项目与源码 | 实际数据和执行方式 | 借鉴与取舍 |
| --- | --- | --- |
| 官方 SDK2 Python：[Arm7 示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py) | `arm_joints`包含双臂 14 关节和腰 Yaw/Roll/Pitch；按周期更新 q，运动阶段写 `dq=0`，结束渐降 J29.q 权重。源码注明锁腰版本的腰 Roll/Pitch 无效。 | 作为上肢 17 关节映射和高层通信依据；示例并非拖动示教器，也不证明低增益腰部示教已验收。 |
| 官方 SDK2 C++：[双臂轨迹例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_dual_arm_example.cpp)、[motion.seq](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/behavior_lib/motion.seq) | 加载角度帧，按固定周期回放，目标 `dq=0`；使用全身低层通道。 | 借鉴位置序列与时间的表达，不移植关闭运动服务及 `rt/lowcmd` 路径。 |
| 官方 XR：[录制内容](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/teleop_hand_and_arm.py#L402-L474)、[手臂控制器](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm.py#L141-L191) | 反馈读取 q/dq；该录制代码保存 qpos，qvel 列表为空；控制器另行发送位置、零目标速度和力矩。 | 区分实测状态、动作目标和设备发送；不复制空字段或训练数据层级。 |
| 社区 Arm7：[teach.cpp](https://github.com/lez666/g1-arm-teach-play/blob/main/teach.cpp)、[play.cpp](https://github.com/lez666/g1-arm-teach-play/blob/main/play.cpp) | Enter 抓取双臂位置路点；播放线性插值并发 `dq=0`；多动作依赖共同 GUARD 姿态，没有避碰规划。 | 借鉴直观入口；定点示教的无时间路点不能替代本项目连续录制。 |
| 社区 G1：[record.py](https://github.com/GetSoloTech/unitree-g1-replay/blob/main/g1_record_replay/record.py)、[replay.py](https://github.com/GetSoloTech/unitree-g1-replay/blob/main/g1_record_replay/replay.py)、[设备接口](https://github.com/GetSoloTech/unitree-g1-replay/blob/main/g1_record_replay/core/g1_interface.py) | 记录位置、速度、时间；回放只取位置和时间；未传目标速度时接口用零值。 | “记录 dq”不等于“回放使用 dq”；不复制 HDF5 元数据、安全问答和全身控制接口。 |
| 社区 Arm5：[drummer 录制](https://github.com/Uoghluvm/unitree-g1-drummer/blob/main/scripts/g1_arm5_teach_record.py)、[回放](https://github.com/Uoghluvm/unitree-g1-drummer/blob/main/scripts/g1_arm5_drum_replay.py) | 原始记录含 t/q/dq/力矩；回放只使用 t/q，重采样后发 `dq=0`。 | 借鉴返回播放前姿态的生命周期；不套用 Arm5 下标、参数或打鼓处理。 |

没有“G1 动作文件必须存 dq”的统一惯例。是否保留 dq 要看是否需要重现实测速度，不能照着反馈报文字段复制动作格式。

### 2.2 衔接、规划与代码组织

| 第一方资料 | 查证事实 | 本项目采用什么 |
| --- | --- | --- |
| 官方 UnifoLM：[设备组织](https://github.com/unitreerobotics/unifolm-world-model-action/blob/main/unitree_deploy/docs/add_robot_arm.md)、[关节插值器](https://github.com/unitreerobotics/unifolm-world-model-action/blob/main/unitree_deploy/unitree_deploy/utils/joint_trajcetory_inter.py) | 设备模块集中连接、读写与回位，插值器处理时间与位置，没有几何避碰。 | 设备会话与数值轨迹分开，不复制多机型工厂与注册表。 |
| [ROS2 轨迹控制器](https://control.ros.org/kilted/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html) | 普通位置插值为线性，只保证位置连续；位置和速度可用于三次插值。不一致导数可能造成超调。 | 从同一曲线产生位置与速度，不将逐帧发位置等同于平滑；不引入 ROS2。 |
| [MoveIt 轨迹处理](https://moveit.picknik.ai/main/doc/concepts/trajectory_processing.html)、[时间参数化](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html) | 几何路径与时间轨迹分开；部分重采样改变空间路径，需要额外碰撞检测。 | 规划与时间安排分工，不复制整个框架和速度/加速度参数体系。 |
| [OMPL PathSimplifier](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathSimplifier.html) | 提供保持路径有效的快捷连接与简化。 | 使用 RRTConnect 和带碰撞判断的删点，不另写搜索算法。 |
| [SciPy CubicSpline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html) | 时间与位置可构造 C2 三次样条，clamped 约束两端速度为零，可直接求导。 | 用成熟数值库生成 q(t)/dq(t)，不手写样条求解器。 |

## 3. 本项目的明确设计

### 3.1 文件只存 t/q，执行轨迹同时产生 q/dq

| 数据 | 来源 | 用途 |
| --- | --- | --- |
| 实测 q | SDK 反馈中所选关节的位置。 | 连续录制保存。 |
| 实测 dq | SDK 反馈速度。 | 不是本次动作文件或回放输入。 |
| 目标 q/dq | 最终插值曲线及其导数。 | 统一采样后用于回放、衔接与回位。 |

动作文件不存 dq；内存 `Trajectory.sample(t)`统一返回上肢 17 维 q/dq。四种动作组使用同一数值实现，非选关节保持启动位置、目标速度为零。

实际播放将曲线速度填入 `MotorCmd.dq`，不复制反馈 dq。这是本项目选择，与官方入门例固定 `dq=0` 有区别，不表述为厂商要求或真机验证结论。通过位置重建的速度也不声称等于原始实测 dq。

`--speed=s`时，动作取 q(s*t)，目标速度同步乘 s，时长除以 s。速度只在统一采样入口处理一次。

### 3.2 一种动作格式

以下为双臂加腰的两帧结构示例：

```json
{
  "group": "arms_waist",
  "frames": [
    {"t": 0.0, "q": [0.0, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {"t": 0.02, "q": [0.01, 0.2, 0.0, 0.81, 0.0, 0.0, 0.0, -0.01, -0.2, 0.0, 0.81, 0.0, 0.0, 0.0, 0.01, 0.0, 0.005]}
  ]
}
```

- `group`取第 1 节四个值，直接决定关节映射和 q 的维数，不另加 enable 或控制布局字段。
- `t`是实际相对秒，首帧为 0，随后递增；不重复保存 dt、fps 或 duration。
- 每侧手臂七维顺序为肩 Pitch、肩 Roll、肩 Yaw、肘、腕 Roll、腕 Pitch、腕 Yaw。
- `arms`按左臂七维、右臂七维排列；`arms_waist`在这十四维后追加腰 Yaw、Roll、Pitch。
- 至少两个时间不同的样本构成连续动作，不另建 waypoint 格式。

文件只有 group 和 frames，每帧只有 t/q。没有 side、dq、ddq、力矩、schema、target_profile、joint_names 副本、notes、qualification、哈希、名字或配置副本。文件名就是标识。

加载后从本次启动的 17 维上肢姿态复制保持目标，再覆盖本组位置。这是明确的未选关节保持语义，不是为旧格式缺失关节补零。旧 side 格式不适配、不转换。

### 3.3 一份增益配置

```json
{
  "arms": {
    "hold_kp": 60.0,
    "hold_kd": 1.5,
    "teach_kp": 20.0,
    "teach_kd": 1.5
  },
  "waist": {
    "hold_kp": 60.0,
    "hold_kd": 1.5,
    "teach_kp": 20.0,
    "teach_kd": 1.5
  }
}
```

只保留实际进入报文的八个增益值。数值用于展示结构，不代表当前设备已验证参数。

| 阶段 | 所选手臂 | 未选手臂 | 腰部 |
| --- | --- | --- | --- |
| 接管、固定保持 | arms.hold | arms.hold | waist.hold。 |
| 示教与等待回车 | arms.teach，目标随反馈。 | arms.hold，保持启动目标。 | group 为 arms_waist 时用 waist.teach 并随反馈；其余用 waist.hold 并保持启动目标。 |
| 结束录制后的保持 | 渐变恢复 arms.hold。 | arms.hold。 | 参与示教时渐变恢复 waist.hold，否则始终 waist.hold。 |
| 回放、衔接、回位、交回 | arms.hold。 | arms.hold。 | waist.hold。 |

腰部独立配置保持和示教增益，不把手臂示教增益直接套到腰上。也不分别维护单臂、双臂、带腰及回放的多份配置。

机型、SDK 槽位、控制周期与资源路径集中在代码定义；接管、增益渐变和交回的固定流程时长各只定义一次，不扩展为参数大全。

### 3.4 启动上下文与统一控制域

离线 `start.json`只有 `leg_q`（左腿六维、右腿六维，共 12 维）和 `upper_q`（左臂七维、右臂七维、腰三维，共 17 维）。腰不重复出现在另一份 body 数组中。

实际执行从本次 SDK 反馈取得相同上下文。启动时固定一次；回放不能把录制时的旧姿态当作当前姿态。

程序发送目标和内部轨迹统一为上肢 17 维。SDK 对应槽位集中映射为左臂 J15–J21、右臂 J22–J28、腰 J12–J14；J29.q 是唯一上肢控制权重，不属于动作关节。腿部反馈只用于模型上下文，不向腿部发送位置控制目标。

### 3.5 规划与范围切换

| 连接位置 | 允许搜索和运动的关节 | 其余上肢关节 |
| --- | --- | --- |
| 启动点 → A 起点 | A 的 group。 | 固定本次启动目标。 |
| A 终点 → B 起点 | A.group 与 B.group 的并集。 | 固定本次启动目标。 |
| 最后动作 → 启动点 | 最后动作的 group。 | 已处于启动保持目标。 |

活动关节由 group 决定，不能只比较起终点判断；选中关节即使两端角度相同，也可能需要参与绕障。内部输出始终 17 维，不允许为了单臂绕障而擅自移动未选右臂或腰。

例如 `arms_waist → left_arm`：连接段允许双臂和腰运动，结束时右臂与腰回到本次启动目标，左臂进入新起点；随后的左臂动作只动左臂。`left_arm → right_arm`的连接允许双臂运动，腰始终保持。

1. **动作段：** 实际 t/q 经 `CubicSpline(..., bc_type="clamped")`生成；整段两端速度为零，内部采样点不强制停下。
2. **连接段：** 先尝试直连，有碰撞才调用 OMPL RRTConnect；按同一碰撞规则快捷删点。
3. **连接时间：** 折线逐段采用单调五次时间标量，按段长度分配总时长；保留空间线段，在拐点速度降为零。首版允许绕行转角停顿。
4. **完整序列：** 启动点 → A → 过渡 → B → 启动点；动作和连接边界 q/dq 连续，不承诺不停顿融合或边界加速度连续。
5. **碰撞：** 将固定 leg_q 与动态 upper_q 一起做 FK，沿最终曲线及帧间加密采样。腰参与运动时躯干、头部和双臂随腰更新，检测覆盖这些部件与腿及其他身体部件的碰撞，不能只筛手臂几何。

检测针对同一条最终播放曲线，不能只检查原始端点或 RRT 折线；离散检测不称作连续空间无碰撞证明。腿不参与搜索，不等于腿的几何不参与检测。

`--transition-seconds`安排连接耗时，`--speed`安排动作倍率，不是数值达标门槛。不恢复 max_v/max_a 拒绝规则，不增加 TOTG、Ruckig、在线重规划或第二套平滑管线。

模型以官方 [G1 29DoF 资源](https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/g1/g1_29dof.xml)为基础，保存采用版本与许可；运动变量覆盖上肢 17 关节，腿作为固定上下文。只使用随包固定末端几何，不提供可动手指、手型配置或包络，按模型实际范围说明覆盖内容。

### 3.6 示教、回放与退出

**示教：** 选择 group → 读取启动反馈 → 保存启动姿态 → 权重渐变接管 → 所选关节渐变到各自示教增益并跟随反馈 → 第一次回车开始 → 同步记录本组 t/q → 第二次回车结束并冻结终点 → 保存动作 → 恢复各自保持增益 → 规划返回 → 执行返回 → 权重降到 0 → 关闭。

第二次回车时结束录制与固定终点使用同一反馈快照，文件写入期间不继续反馈跟随。保存先于回位；回位无解仍保留动作。正常结束返回启动时的上肢 17 关节目标，不是 29 关节全身回位。

示教跟随阶段目标 q 取选中关节的实测位置，目标 dq 为零；未选上肢固定保持。回放、衔接和返回使用最终曲线 q/dq。工具不提供重力补偿或动态平衡求解。

**回放：** 加载动作列表 → 读取本次启动反馈 → 按各动作 group 构造完整序列 → 一次接管 → 按单调时钟连续发送上肢 q/dq → 最后返回目标发出 → 降权重 → 关闭。

**退出：** 正常结束自动返回；Ctrl+C、EOF、规划无解走统一收尾，不另起自动回家动作。已开始控制且发送仍可用时固定当前目标、目标 dq 为零并渐降权重；SDK 明确写入失败直接报错，不伪造释放成功。

只有一个发送循环。输入只产生事件，规划只返回轨迹，二者不直接写命令；规划期间循环持续保持目标。输入与规划工作可结束、可回收，不遗留阻塞 stdin 的 daemon 线程。

## 4. 用户故事

### US-001：安装与单一入口

**描述：** 作为开发者，我希望安装后在任意目录使用同一个命令入口。

**验收标准：**
- [ ] 安装入口和模块入口调用同一 main，常用命令只有 teach、preview、replay。
- [ ] 从项目目录外加载动作和随包模型资源成功。
- [ ] 帮助、文件读取和离线预览不初始化 DDS。

### US-002：选择示教范围并直接使用动作文件

**描述：** 作为使用者，我希望选择单臂、双臂或双臂加腰，录制产物直接预览和回放。

**验收标准：**
- [ ] teach 的 group 支持 left_arm、right_arm、arms、arms_waist 四个值。
- [ ] 四组动作每帧 q 分别为 7/7/14/17 维；文件只有 group/frames，每帧只有 t/q。
- [ ] 预览和回放从文件读取 group，不要求再次指定，也不拆成多份文件同步。
- [ ] 只在入口一次解析必要字段、与 group 对应的有限数向量及递增时间。
- [ ] 无旧 side 格式、版本归一化、补零、截断或资格判断。

### US-003：统一生成目标位置与速度

**描述：** 作为维护者，我希望四种动作范围共享最终曲线和时间采样。

**验收标准：**
- [ ] `Trajectory.sample(t)`始终返回上肢 17 维 q/dq，报文使用相同采样结果。
- [ ] 数值验证运动段 dq 与 q 的导数一致，非选关节 q 保持启动值、dq 为零。
- [ ] speed=0.5 时动作时长加倍，对应位置不变、速度减半。
- [ ] 动作两端速度为零，内部运动样本速度非零，不是每帧停一次。
- [ ] 连接与动作边界 q/dq 连续，绕行拐点允许停顿，不要求 ddq 连续。

### US-004：两次回车完成多关节同步示教并返回

**描述：** 作为示教者，我希望同时拖动所选关节，按两次回车完成录制和自动回位。

**验收标准：**
- [ ] 单臂模式只让该臂跟随，双臂模式让双臂同时跟随且腰保持。
- [ ] arms_waist 模式同步记录双臂和腰，并分别采用 arms.teach 与 waist.teach。
- [ ] 未选关节始终使用各自 hold 增益和启动目标。
- [ ] 两次回车之间记录实测 t/q；接管、增益切换和回位不录入动作。
- [ ] 第二次回车在同一快照下结束记录并冻结终点，保存文件后恢复保持增益与规划回位。
- [ ] 正常返回启动 upper_q[17]，回位无解仍保留动作文件。

### US-005：模型和规划覆盖动态腰部

**描述：** 作为动作编排者，我希望预览、避碰和发送采用相同上肢姿态与范围。

**验收标准：**
- [ ] 进入和返回的搜索只允许当前 group，动作间连接只允许前后 group 的并集。
- [ ] 覆盖 arms_waist→left_arm：连接结束右臂和腰回启动目标，左臂进入下一动作。
- [ ] 模型使用 leg_q[12]+upper_q[17]；改变腰角会改变躯干、头部和双臂的 FK 与碰撞结果。
- [ ] 碰撞检测覆盖这些运动部件与固定腿及其他身体部件，不沿用仅手臂碰撞对筛选。
- [ ] 实际模型测试覆盖直连失败而绕行成功、无解、端点不碰而中间碰撞。
- [ ] 预览加载离线起点，显示同一最终曲线，不默认零姿态，不创建控制发布器。

### US-006：一次接管连续回放不同范围的动作

**描述：** 作为操作人员，我希望按顺序传入不同 group 的动作，一次连续播放并结束。

**验收标准：**
- [ ] 单动作与多动作使用同一序列函数，接管前规划整段。
- [ ] 整段只接管、返回、交回一次，共用 J29.q 权重，动作间不另开会话。
- [ ] 每段手臂使用 arms.hold，腰使用 waist.hold；是否运动由 group 与轨迹决定。
- [ ] 最后返回目标发出后交回，不进入 settle 或实测误差达标等待。
- [ ] Ctrl+C、EOF 与规划异常可结束并回收工作；发送与收尾没有多个所有者。
- [ ] 超过旧误差、速度、起点偏移、身体漂移或循环间隔阈值不触发对应停止分支。

### US-007：自动验证四种范围的完整使用流程

**描述：** 作为维护者，我希望不连接机器人就能验证示教、规划、预览与混合序列回放。

**验收标准：**
- [ ] 自动用回车录制左臂、右臂、双臂和双臂加腰动作，直接加载生成文件。
- [ ] 用 arms_waist 和 left_arm 产物完成真实规划、预览数据构建与连续回放。
- [ ] 使用实际模型、SciPy 和 OMPL，只替换 SDK、时钟和输入，不替换整个规划函数。
- [ ] 断言范围切换、q/dq 一致、固定腿上下文、未选关节保持、最终 upper_q[17] 返回和末次权重为 0。
- [ ] 验证腰参与示教时的反馈录制与独立 teach/hold 增益切换。
- [ ] 覆盖取消和回位无解，验证直接错误结果、动作保留与资源退出。
- [ ] 官方 SDK 消息类型离线检查上肢 17 关节映射、q/dq、权重槽和 CRC，不初始化 DDS。
- [ ] Linux CI 完整执行且独立准备和清理数据；缺少规划依赖不得以跳过充作通过。

## 5. 功能要求

- FR-1: 系统必须支持 left_arm、right_arm、arms、arms_waist 四种示教范围。
- FR-2: 系统必须把所选关节组的同步实测位置和相对时间写入同一种动作文件。
- FR-3: 系统必须让录制文件直接作为预览和回放输入。
- FR-4: 系统必须从最终曲线统一采样上肢 17 维目标 q/dq。
- FR-5: 系统必须在动作变速时同步调整目标速度。
- FR-6: 系统必须让当前动作未选中的上肢关节保持本次启动目标。
- FR-7: 系统必须分别配置手臂与腰部的保持和示教增益。
- FR-8: 系统必须根据前后动作组决定连接段允许运动的关节集合。
- FR-9: 系统必须用同一连接函数规划进入、动作衔接与返回。
- FR-10: 系统必须将动态腰姿纳入 FK 和最终曲线的帧间加密碰撞检测。
- FR-11: 系统必须在一次设备会话中连续执行不同范围的输入动作。
- FR-12: 系统必须在正常结束时返回本次启动的上肢 17 关节目标。
- FR-13: 系统必须为正常结束和取消共用权重交回实现。
- FR-14: 系统必须删除旧回放阈值和 settle 分支。
- FR-15: 系统必须避免将报告或资格文件作为执行门槛。
- FR-16: 系统必须移除所有灵巧手数据与控制设计。
- FR-17: 系统必须在离线入口保持设备通信未初始化。
- FR-18: 系统必须通过不连接机器人的完整流程验证。

## 6. 目录与直接调用关系

```text
g1-action-studio/
├── pyproject.toml
├── uv.lock
├── README.md
├── src/g1_action_studio/
│   ├── __init__.py
│   ├── __main__.py
│   ├── cli.py        # 参数、增益配置读取、命令分发
│   ├── motion.py     # Action读写、时间曲线、Trajectory.sample(t)
│   ├── model.py      # 唯一关节/分组映射、模型与路径碰撞
│   ├── planner.py    # 按活动关节直连/搜索、完整序列构建
│   ├── robot.py      # SDK、反馈、唯一发送循环与交回
│   ├── teach.py      # 所选范围同步录制与回位编排
│   ├── replay.py     # 单动作与不同范围序列回放
│   ├── preview.py    # 显示同一模型与轨迹
│   └── resources/    # 固定模型、网格及来源
├── tests/            # 曲线、动态腰几何、报文与完整流程
├── examples/         # 一份配置、四种范围动作、离线启动示例
├── tasks/            # 本设计及历史需求
├── docs/             # 必要说明与设备字段依据
└── data/             # 忽略的现场数据、日志与重写前备份
```

不因支持四个 group 增加四套执行器。motion 统一动作与时间曲线；model 统一分组、关节映射与几何；配置只做直接读取，不单建配置框架。

```text
cli → teach / replay → RobotSession
teach / replay / preview → plan_sequence → connect(active_groups) → model
plan_sequence → motion.Trajectory
RobotSession → Trajectory.sample(t) → 上肢17维q/dq映射 → Write()
preview → 同一Trajectory与model
```

RobotSession 管设备状态与周期发送；Action 是加载一次的数据；Trajectory 是只读曲线组合。其余优先函数，不预建 Service、Manager、Repository、Port、工厂或插件层。

model 模块先定义轻量映射，创建模型时再加载 MuJoCo；robot 实际连接时才导入 SDK。规划不依赖设备，设备不调用搜索，连续回放编排不在 CLI 重复实现。

### 使用命令

以下是待实现的新契约，`--group`替代旧 `--side`，不保留别名：

```bash
g1-action-studio teach --interface eth0 --group left_arm --config config.json --output left.json --execute
g1-action-studio teach --interface eth0 --group right_arm --config config.json --output right.json --execute
g1-action-studio teach --interface eth0 --group arms --config config.json --output arms.json --execute
g1-action-studio teach --interface eth0 --group arms_waist --config config.json --output upper.json --execute

g1-action-studio preview upper.json left.json --start start.json
g1-action-studio replay upper.json left.json --interface eth0 --config config.json --execute
```

preview/replay 共用 `--speed`（默认 1）和 `--transition-seconds`（默认 2）；teach 回位接受同一过渡时长参数。`--interface`仅实际连接时需要；缺少 `--execute`不连接设备，直接提示用法，不跑预检链。

增益配置只由 teach/replay 使用，preview 不要求控制配置。回放不要求报告输出路径。预览是可选能力，不是执行门槛；不要求先运行 build、transition、preflight、audit 或生成资格文件。

## 7. 删除内容与非目标

| 删除范围 | 具体处理 |
| --- | --- |
| 灵巧手 | 删除手部模块、DDS 通道、手指帧、手型、手部增益、包络及排除开关，不留占位适配。 |
| 文件冗余 | 删除 schema、target_profile、notes、qualification、哈希、source、指标及配置副本。 |
| 旧格式 | 删除 side+q7 的唯一单臂契约，统一为 group+对应维度；不兼容旧 side、旧配置或 v1/v2。 |
| 阈值规则 | 删除 max_target_velocity、max_target_acceleration、max_tracking_error、initial_pose_tolerance、return_tolerance、return_speed、state_timeout、max_tick_gap、settle_timeout 及旧硬编码同类分支。 |
| 特殊恢复与重复管线 | 删除 recovery_release_s、独立恢复执行器、录制转换、重复加载/归一化、逐层校验和多层执行包装。 |

旧阈值不得改名、藏入常量或通过 unsafe 开关保留。SDK 明确失败、Ctrl+C、EOF、文件不可解析和规划无解仍返回直接结果；几何分辨率和搜索终止条件只用于规划算法。

非目标：灵巧手、腿部或全身动作示教、步态与动态平衡控制、锁腰机型和 Arm5 兼容、多机型框架、ROS2 集成、Web 界面、远程守护、训练数据集、实测速度保真、不停顿融合、动力学最优和自动切换厂商运动服务。

腰部示教已纳入本次范围，不能继续沿用“所有阶段腰部固定”或“腰部拖动不做”的旧结论。几何规划不求解机体动态平衡；该边界写在操作文档，不增加运行资格字段或阈值流程。

## 8. 先删除再重写与完成标准

1. 先完成本 PRD 的范围更新，再据此生成技术 SPEC。当前不删除或修改运行代码。
2. 实施前在忽略的 data/rebuild-backup/ 独立保存当前源码、未提交变更和未跟踪文件；保留现场录制、取证材料、适用模型和许可，不能只依赖 Git 恢复未提交内容。
3. 分离模型后，删除旧项目全部生产 Python 源码、旧测试、自编诊断脚本和构建产物。范围限项目实现，保留 .git、.venv、数据和第三方 SDK。
4. 从空的标准包目录按新版设计新写，不搬旧模块，不留兼容层、旧入口或第二套执行器。
5. 新写四种范围的示例、测试与 CI，改写 README/CONTRIBUTING；历史文档标明不适用新版。
6. 完成离线流程与安装验证后交付实现，本轮不执行真机动作。

实施顺序：motion/model → planner → robot → teach/replay → preview/CLI → 完整流程。支持更多关节组只扩展同一映射与活动范围，不增加平行管线。

| 成功指标 | 验收证据 |
| --- | --- |
| 范围完整 | 左臂、右臂、双臂、双臂加腰均能录制并直接回放。 |
| 数据精简 | group+frames 一种格式；t/q 为实测，执行 q/dq 来自同一最终曲线。 |
| 未选保持 | 同侧连接不移动另一臂或腰；范围切换按组并集衔接。 |
| 动态腰一致 | 腰部记录、目标、模型 FK、预览与碰撞使用同一动态姿态。 |
| 结束明确 | 正常返回启动 upper_q[17]，末次权重为 0；腿部从不由本工具控制。 |
| 结构清楚 | 八个实际业务模块；分组映射、发送循环和序列构建各只有一份。 |
| 旧复杂度移除 | 无灵巧手、旧阈值、settle、资格文件或格式兼容分支。 |
| 完整流程可复现 | US-007 在 Linux CI 运行实际规划依赖通过，项目目录外安装验证通过。 |

以上验证代码和离线几何，不把公开项目、CRC 或 SDK 写入成功等同于当前设备动作验收。

## 9. 后续技术规格需要落实的内容

技术 SPEC 需要冻结四组的唯一关节映射、17 维轨迹接口、活动关节搜索、动态腰碰撞对、同步录制时序、增益渐变、SDK 报文和统一交回。当前设备采用的消息字段、实际增益和固定末端几何范围，以固定官方 SDK、模型版本及设备资料为依据，不增加配置资格标签。

产品范围已经明确，无需再询问是否支持双臂或腰、是否保留避碰、是否兼容旧格式。下一步依据本 PRD 更新技术 SPEC；需要拆实施任务时使用 to-issues，不建立额外文档流水线。

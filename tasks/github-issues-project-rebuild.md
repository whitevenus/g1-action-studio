# G1 Action Studio 重写：GitHub Issues 发布记录

来源：[更新后的 PRD](prd-project-rebuild.md) 与 [技术 SPEC](spec-project-rebuild.md)。目标仓库：[whitevenus/g1-action-studio](https://github.com/whitevenus/g1-action-studio)。日期：2026-10-16。

状态：**已按用户确认发布，并核对正文与原生依赖**。下文 01–05 保留为计划编号，实际 GitHub Issue 为 #9–#13；远端正文中的跨票引用已替换为实际 Issue 编号。

发布前已核对：旧 GitHub Issues #1–#7 均已关闭，没有开放的同类重写任务。它们描述的旧兼容与手部方案不作为本次实现依据。

| 计划 | GitHub | 可演示行为 | 直接依赖 | 验收归属 |
| --- | --- | --- | --- | --- |
| 01 | [#9](https://github.com/whitevenus/g1-action-studio/issues/9) | 删除旧实现后，从新格式离线预览四种范围的直连动作序列。 | 无 | 新入口、文件、数值曲线、真实模型、直连预览及初次安装验证。 |
| 02 | [#10](https://github.com/whitevenus/g1-action-studio/issues/10) | 同一预览命令对受阻连接自动绕行。 | #9 | 真实 RRTConnect、活动范围、精确解、删点与规划取消。 |
| 03 | [#11](https://github.com/whitevenus/g1-action-studio/issues/11) | 一次接管连续回放，正常返回并交回；取消和失败统一收尾。 | #10 | SDK 报文、会话、回放 CLI、发送与输入生命周期。 |
| 04 | [#12](https://github.com/whitevenus/g1-action-studio/issues/12) | 两次回车录制任意范围，保存后避碰回位。 | #11 | 四组同步反馈记录、原子冻结、八增益、保存及示教 CLI。 |
| 05 | [#13](https://github.com/whitevenus/g1-action-studio/issues/13) | 在干净安装中使用四组录制产物完成混合预览与回放。 | #12 | 产物贯通、完整 Linux CI、包内资源及最终使用文档。 |

依赖为 `01 → 02 → 03 → 04 → 05`。03 包含唯一会话，04 复用它；不为并行而增加独立 SDK 层、临时适配器或第二套执行流程。每张功能票交付自己的测试与使用说明；05 不承接前面未完成的功能或测试。

已验证 GitHub 原生链：`#9 → #10 → #11 → #12 → #13`，箭头表示前项阻塞后项。#9 无前置阻塞，#13 无下游阻塞；五张均为 OPEN 并带 enhancement 标签。正文与确认草稿一致，仅将计划引用替换为实际 Issue 引用。

01 的起点是当前尚未重写的工作树，后续各票以其前置票完成的工作树为起点；每张验收都要求本票新增的完整路径或交付产物，不能仅凭已存在的接口、测试数量或重复执行前置用例通过。

01 是一个完整的离线直连切片：受阻连接先明确报规划错误，02 在同一个 connect 中补齐绕行。没有临时命令、兼容格式或 straight-only 选项。设备入口在 02 完成后才由后续票提供。

五张均使用现有 enhancement 标签，优先级 high 写入正文。未新建标签、父 Issue、里程碑或项目；本轮完成任务发布，实际实施由这些 Issues 推进。

---

## 01 · 从空包重写，并离线预览四种范围的直连动作序列

### Description

先独立备份当前工作，再删除旧项目实现，从空的标准 Python 包完成新格式动作到真实模型预览的第一条完整路径。支持左臂、右臂、双臂及双臂加腰；当前切片对合法直连完成进入、动作衔接和返回，受阻时明确报规划错误。后续绕行票扩展同一个连接函数。

### Demo path

从项目目录外运行安装后的 preview 命令，依次载入四种范围及一个 arms_waist→left_arm 序列；查看器显示进入、动作、直连衔接和返回启动姿态。半速播放时动作时长加倍、目标速度减半，连接时间不变，全程不初始化 SDK 或 DDS。

### 实施合同

- **删除先于重写。** 将现有源码、未提交及未跟踪文件、测试、脚本和文档保存到 SPEC §10 指定的忽略备份区，核对备份后再删除旧生产 Python、旧测试、自编诊断脚本、旧示例及构建产物。任何删除前解析并核实绝对目标在本仓库内。保留 Git 元数据、虚拟环境、现场数据、第三方 SDK 及适用许可。备份不进入新运行入口，不搬旧模块继续改造。
- 标准 src 包布局，包名 g1_action_studio；本票实现 cli、motion、model、planner、preview，后续再增加 robot、replay、teach。只有八个最终业务模块，不引入 Service、Port、Manager、插件注册、兼容器或重复 SDK 包装。删除旧构建映射和入口；每个当前公开命令均有实际实现，不用成功返回的占位命令。
- Python >=3.11；NumPy >=1.26,<3、SciPy >=1.16.3,<1.17；planning 使用 MuJoCo >=3.15,<4，Linux OMPL ==2.0.1。沿用 unittest、Ruff 和 uv 锁定依赖，不顺带升级无关依赖。删除旧实现时同步移除失效的旧 CI 调用，以本票实际文件/数值/模型测试维持可运行的最小 CI，完整 SDK 与跨命令 CI 由05交付。
- 唯一关节表派生分组与 SDK 映射：upper_q 顺序为左7、右7、腰 Yaw/Roll/Pitch 3，对应 SDK 槽位 15–28、12–14。left_arm/right_arm/arms/arms_waist 的文件维数依次 7/7/14/17；腿12仅为固定模型上下文，J29 权重不是关节。按模型关节名查实际 qpos 地址，不能把 SDK 槽位当 qpos 下标。
- 动作仅为 `{"group":"left_arm","frames":[{"t":0,"q":[0,0,0,0.8,0,0,0]},{"t":0.02,"q":[0.01,0,0,0.81,0,0,0]}]}` 这一结构（数值只示格式）；起点仅 leg_q[12]、upper_q[17]。Action 的 t/q 使用只读 float64 数组，至少两帧、首时刻为0且严格递增。只在读取边界检查精确键集合、维度和有限数字，bool 不当数字；拒绝 side、v1/v2、手部、notes、qualification、schema 及其他旧字段，不迁移。保存只写 group/frames/t/q，UTF-8、末尾换行、独占创建，已有文件不覆盖。
- `load_action / save_action / load_start` 与 `action_curve(action,start_upper,speed)`；`connection_curve(points17,seconds)`；`concatenate(parts)` 由同一个 motion 实现。Trajectory 只有一个 PPoly、duration 和精确首末目标，`sample(t)` 返回 q17/dq17。未选关节恒为启动目标；区间外返回精确端点和 dq=0。
- 动作使用 `CubicSpline(t/speed,full_q,axis=0,bc_type="clamped",extrapolate=False)`；仅整段两端速度为零，内部不逐帧停顿，不在发送端再乘 speed。三次系数最高两阶补零，与五次连接统一为 shape(6,intervals,17)；拼接仅平移断点、连接系数，不重拟合路径，内部边界取后一段。
- 连接活动维度为：进入/返回取当前组，A→B 取两组并集，其余关节固定启动目标。有效重合端点返回 None 且不占时；合法直连使用五次 h(u)=10u³−15u⁴+6u⁵，dq=30u²(1−u)²Δq/T。`plan_sequence(actions,start,model,speed,transition_seconds,cancel)` 构造进入→所有动作及衔接→返回，拼成唯一轨迹并检测。直连受阻暂报 PlanningError，不绕过检测；不添加新开关或第二条管线。
- 模型资源采用官方 unitree_mujoco 的 G1 29DoF、历史版本 ab03eec，取资产时解析完整 SHA，随包保留来源与许可证及所需网格；不复用旧 Dex3 模型，不运行时下载。自由基座保留模型初始位置和单位四元数；每次更新固定腿12和动态上肢17后做 mj_forward。
- MuJoCo 碰撞只用实际碰撞 geoms，尊重掩码和装配排除，忽略显示副本；按祖先关节识别上肢影响的几何，头、躯干、双臂必须随腰变化。接触至少一端受上肢影响且距离<=0即碰撞，固定腿仍参与；不新增 clearance 配置。
- 几何采样分辨率0.02rad。线段包含两端，分段数 `N=max(1,ceil(||b-a||∞/0.02))`；最终 PPoly 每段通过端点和加速度实根求最大绝对速度 M，分段数 `N=max(1,ceil(M*区间时长/0.02))`，均分加密并含全部 knots。关节 range 还检查位置多项式在端点及 dq=0 实根处的极值。动作碰撞/越界报错，不改写动作、裁剪目标或改成直线。检测仅属于规划，不放进发送周期。
- `preview <动作列表> --start <启动姿态>` 使用同一 plan_sequence；speed 默认1、transition-seconds 默认2，均为正有限数。preview 不需要增益/网卡，不创建 RobotSession、不初始化 SDK，模型与 viewer 各持自己的 MjData；按轨迹写 q17+forward 显示，不用 mj_step 替换曲线。关闭查看器退出；无显示测试只声明绘制数据验证。
- 模型创建时才导入 MuJoCo；轻量映射可被 motion 导入，model 不反向导入 motion，曲线检测在 planner，避免循环。文件、用法和规划错误遵循 SPEC §8；取消能退出几何循环。没有旧回放阈值、资格文件或灵巧手占位。

### Acceptance Criteria

- [ ] 留下已核对的独立备份和删除清单；新包不导入或执行备份及旧实现。安装后能完成本票新格式预览路径，旧入口不能继续执行旧代码；发现旧代码仍是运行依赖即失败。
- [ ] 四组文件实际加载为7/7/14/17维，并写回精简字段；错误维度、非递增时间、bool/非有限数和任意旧字段在入口失败。已有输出内容保持原样。
- [ ] 真实 SciPy 数值用例证明 q/dq 与数值差分一致，0.5倍速度使动作时长变2倍且 dq 变0.5倍；首末 dq=0、内部存在非零 dq、非活动关节恒 home。PPoly 拼接前后各原段位置/导数一致，零长连接不增加断点或时长。
- [ ] 真实 MuJoCo 模型用例证明腰变化影响头/躯干/双臂位姿、固定腿参与碰撞；能拒绝端点无碰撞但中段碰撞的曲线，以及采样点合法但多项式极值越界的曲线。若仅检测原始动作帧或沿用固定腰模型即失败。
- [ ] 安装后的 preview 对合法四组及混合序列实际调用同一序列函数，完整返回 upper_q[17]；arms_waist→left_arm 之后右臂/腰保持 home，left_arm→right_arm 连接腰保持 home。受阻直连明确失败，绝无放开非活动关节或跳过检测的兜底。
- [ ] wheel 包含固定 XML、网格、来源和许可证；在项目目录外实际加载模型、构建预览数据，不依赖源码相对路径。离线帮助/预览在 SDK 不可用且禁止网络构造的环境中仍可运行；无显示环境不得声称已目视验收。
- [ ] 本票随功能交付对应的 unittest、数值/模型证据及简明 preview 用法；使用真实 SciPy/MuJoCo，不整套替换规划函数；不等待最终整合票补上述验证。

### Blocked by

无。

### Priority

high

### SPEC Reference

§1 技术选择；§2 目录与依赖；§3 数据与文件；§4 preview及数值接口；§5.1–5.4 曲线、直连与动态腰；§5.6 预览；§8 错误；§9 T01–T04中本票行为；§10 删除与安装。

---

## 02 · 让受阻动作连接自动绕行，并保持所选关节范围

### Description

扩展已有的 connect：直连碰撞时，使用 OMPL RRTConnect 在前后动作组允许的自由度内找路。用户继续使用同一个 preview 命令，得到避碰的进入、动作衔接和返回；没有精确解时直接失败。

### Demo path

用一组真实模型中直连受阻但存在绕行的动作运行 preview，看到先前失败的连接沿中间路点绕行后到达目标并返回；再换成左臂任务及无解任务，确认其余关节不被放开、无解不被直线代替。

### 实施合同

- 复用唯一 Action、StartPose、Trajectory 和模型：四组维数7/7/14/17，轨迹始终左7+右7+腰3共17维；模型固定腿12、动态上肢17，无灵巧手、旧文件兼容或控制阈值。
- 保持 `connect(model,a,b,active_groups,seconds,cancel)` 合同。先判端点，重合有效返回 None，直连有效仍用两点折线；仅在直连受阻时构造 OMPL RealVectorStateSpace 与 RRTConnect。活动范围只由 group 决定，不按“端点相同”删掉仍可参与绕障的活动关节。
- 进入及返回只开放当前动作组；A→B 开放两组并集。left_arm 只7维、left_arm→right_arm 只14维且腰固定、arms_waist→left_arm 允许17维并在连接末端把右臂和腰送回 home。失败时不能扩大自由度。
- 搜索上下界来自对应 XML range，其他维度固定 home；每个状态和线段复用模型有效性与0.02rad加密。动态腰更新躯干、头、双臂，固定腿仍为障碍。不另起仅手臂碰撞模型。
- 每连接最多5秒搜索预算，只接受精确到达目标的解；近似解、无解和取消分别返回明确失败/取消。算法预算不是状态新鲜度或回放超时参数，不写入配置。
- 得到路径后，在最多1秒且可取消的预算内作保持状态/线段有效的快捷删点；移除连续重复路点，不加 B 样条空间平滑。取消时 C++ validity 回调返回 False，不跨 C++ 边界抛 Python 异常；搜索终止条件、采样及删点均观察同一 cancel。
- 折线段长度 L_i 使用17维欧氏长度，总连接时间 T 按 T_i=T*L_i/ΣL 分配；每段采用五次 h=10u³−15u⁴+6u⁵，q=p_i+hΔp、dq=30u²(1−u)²Δp/T_i。路点处速度为零但不改变已检测的空间折线；一个连接总时长严格为 transition-seconds，不随 speed 缩放。
- 仍然由 plan_sequence 拼接唯一 PPoly，再对实际最终曲线做几何采样和多项式极值范围检测。原动作碰撞或越界不能通过重规划修改；不接受对旧轨迹检测后又拟合新曲线。

### Acceptance Criteria

- [ ] 在真实 MuJoCo/OMPL 的固定、可复现夹具上，同一 preview 输入在前置票因受阻失败，在本票得到精确终点的有效绕行；最终曲线全部通过既有几何/范围检测。不得整套 mock connect 或搜索器来证明绕行。
- [ ] 三类活动范围夹具验证左臂只动7维、左右换组只动14维、含腰换组按17维参与；非活动坐标逐样本等于 home。单臂无解时不出现偷偷放开腰/另一臂的路径；端点相同的活动关节仍可参加绕障。
- [ ] 多路点连接的总时长等于所给 T，位置始终落在原有效线段，路点 q/dq 连续且 dq=0；拼接后不新增空间拟合，不给重合有效端点添加空等候段。
- [ ] 真实搜索用例覆盖确定无解、只有近似结果及搜索中取消：均不返回可播放轨迹，不使用直线兜底，不调用设备接口；取消后不提交已过时结果。针对近似状态可补小范围返回值测试，但不能替代真实成功/失败搜索用例。
- [ ] 搜索、删点和最终几何检查可取消，固定测试在限定算法预算内终止；同一组 preview 用法保持不变，没有新增规划模式、资格报告或 runtime 检查。
- [ ] 本票包含上述回归测试和受阻/无解演示说明；01 的直连预览仍通过，不将绕行或取消测试留给后续设备票。

### Blocked by

计划 01。

### Priority

high

### SPEC Reference

§3.1 分组；§4 connect/check_trajectory/plan_sequence；§5.2 按范围搜索与五次时间函数；§5.3 动态腰和最终检测；§5.4 换组；§6.2 规划取消；§8 规划失败；§9 T04。

---

## 03 · 一次接管连续回放不同范围的动作，并统一交回

### Description

实现 replay 命令和唯一 RobotSession，把已完成的真实规划结果发送为官方 HG 上肢报文。一次接管连续播放所有动作、衔接和返回，再交回权重；本票同时完成正常、取消、SDK失败和部分初始化失败的收尾。

### Demo path

在禁止创建网络实体的离线测试中调用实际 replay CLI，输入 arms_waist→left_arm 的新格式动作。替身通道记录真实官方消息：只有一次接管、17维连续 q/dq、一次回到启动姿态及一次权重降零；取消则固定最后成功目标交回，Write失败后不再写。

### 实施合同

- `replay <动作列表> --interface <网卡> --config <增益> --execute`；speed 默认1、transition-seconds 默认2且正有限。没有 execute 或使用旧参数时退出2并保持设备未初始化。单/多动作只有 `run_replay(interface,actions,gains,speed,transition_seconds,cancel)` 一条路径；新动作仅 group+frames[t,q]，没有兼容转换、报告或预检门槛。
- 配置精确为 arms、waist 两对象，各含 hold_kp/hold_kd/teach_kp/teach_kd，共八个非负有限数，只读取一次。PLAY 使用两组各自 hold 增益；不添加阈值、资格、说明或其他配置字段。
- `with RobotSession(interface,gains,cancel)` 的进入阶段只初始化一次进程级 Factory、创建订阅者并可取消地等待第一条真实反馈；同一 LowState 读取 leg_q12 和 upper_q17，固定一次 start_pose。没有零姿态补值、反馈超时、RPC、模式查询或厂商服务切换。
- 主调用链严格为 `load_action → RobotSession.start_pose → ArmModel → plan_sequence → acquire → play → close`。完整规划在 acquire 前完成，失败没有发布者、控制 Write 或恢复包，不在规划后新增偏差复核。acquire 才创建发布者及唯一非 daemon 发送线程。
- RobotSession 本票提供 start_pose、acquire、play、close；示教票在同一类补 follow、begin_recording、end_recording、hold，不先添加空实现。robot 不调用 planner，planner 不导入 robot，不另设 SDK 绑定器、Port 或 RecoveryExecutor。
- 固定官方 unitree_sdk2_python 提交 `814556d15970dd2ecf1c9984e845ca02ab07e206`。实际 execute 才调用 `ChannelFactoryInitialize(0,interface)`；订阅 `rt/lowstate`，`Init(callback,0)`；发布 `rt/arm_sdk`，使用默认 HG LowCmd 工厂和官方 CRC。
- 唯一 `build_command(q17,dq17,kp17,kd17,weight,crc)`：upper 顺序左7+右7+腰Yaw/Roll/Pitch3；映射到J15–28、J12–14。35槽保留完整；mode_pr/mode_machine、所有 motor.mode/reserve 为官方默认0，受控 tau=0；J0–11保持默认，不写腿反馈；J29只写q权重；J30–34及顶层reserve保持默认。全部字段完成后求官方CRC。PLAY 的活动腰 dq 也来自最终曲线，不能一律置零。
- 状态为未接管→ACQUIRE→PLAY/HOLD→RELEASE→CLOSED。DT=0.02秒，接管/增益/交回共用2秒常量，只有一处发送循环；acquire home/0、weight0→1；play sample同一 PPoly；结束必须精确尾目标成功 Write 后通知主线程。落后按实际单调时钟跳到当前时间，不补发积压、不按 tick 间隔停止。
- 应用只有主线程、一个非 daemon 发送线程和一个非 daemon Linux 输入线程。主线程做文件/模型/规划，发送关键段不做这些工作；短锁复制目标，锁外 CRC/Write，成功后记录 last_q/last_weight。输入 select有限等待+os.read，独立stop可join，不用阻塞 input/readline 或 daemon。
- SIGINT只置 cancel，EOF同样取消。发送线程独立观察 cancel；控制已开始即自行进入 RELEASE，使用最后成功发送 q、dq=0，并从当时 weight 降零、恢复 hold；RELEASE只进入一次，不能每 tick 重新计时。取消不自动回家、不等主线程规划 finally；规划循环使用同一 cancel，取消后不能再 play 旧结果。
- 正常 close 采用独立关闭请求，不设置用户 cancel；幂等，join发送后分别关闭发布者/订阅者，关闭一项失败仍清理其余资源；输入stop/join并恢复信号。SDK没有公开Factory shutdown，不伪造调用。部分初始化失败关闭已拥有资源。
- 一次 Write 返回False/异常后立即停止、不重试交回、不补零权重包；保存原错误并唤醒等待/取消规划。回调实际读数异常也保存错误并cancel，由发送者使用最后成功目标统一交回，不静默吞异常。业务/取消和清理错误并存时保留两者，不能以清理覆盖原原因。
- 成功0、纯用户取消/EOF130、SDK/文件/规划/清理失败1、用法2。不设速度/加速度/误差/起点偏差/返回容差/状态时效/循环间隔/settle/特殊recovery阈值，不改名藏入常量或 unsafe 开关。Write成功只报告本地流程完成，不声称硬件接收或交权验收。

### Acceptance Criteria

- [ ] 实际 replay CLI 使用真实 Action、SciPy、MuJoCo/OMPL 和唯一 plan_sequence 完成单动作与混合列表，通道仅一次接管/交回；精确尾帧 upper_q17 等于启动目标，腿槽默认且腰动作含非零 dq。若调用旧回放器或整套 mock 规划即失败。
- [ ] 固定 SDK 真实35槽消息、官方CRC与serialize测试验证完整映射、默认 mode/reserve、J29仅权重和受控tau；Factory/通道构造用明确禁止联网的替身。测试安装SDK所需CycloneDDS/IDL及本架构CRC库，不以手写假消息替代真实报文证据。
- [ ] 起点反馈未到时可以取消且没有发布者；规划碰撞/无解失败时控制 Write 数量为0；缺少 execute、旧选项或非法速度/连接时间返回2且未初始化Factory。
- [ ] 接管半程取消从当时权重平滑降零，无跳到1；持续置位cancel不会重置RELEASE；EOF和播放取消均可交回并退出。用已接管RobotSession的定向测试证明主线程忙时发送者也能独立交回，不给replay制造接管后规划阶段；实际示教返回规划取消由04验证。取消不执行返回home曲线，正常close不把成功变130。
- [ ] 每个阶段注入一次Write返回False或异常，之后Write次数不再增加；反馈读取异常、部分初始化失败、任一Close失败仍能回收其他已拥有资源，保留业务与清理两种原因，返回1。
- [ ] 输入和发送线程均实际退出并join，信号处理恢复；play只有精确末帧成功发送后才完成。改变反馈偏差、帧间隔等旧阈值因素不产生隐藏停止/settle分支。
- [ ] 本票自带 SDK 报文、完整回放与生命周期测试及简明用法，离线测试没有创建网络实体，没有生产 fake 模式，也不执行机器人动作。

### Blocked by

计划 02。

### Priority

high

### SPEC Reference

§2 模块依赖；§3.1/3.3 映射与增益；§4 RobotSession/replay/CLI；§5.6 完整回放；§6 状态、线程与收尾；§7 官方SDK报文；§8 退出码；§9 T06–T08及对应T01。

---

## 04 · 两次回车同步录制单臂、双臂或双臂加腰，保存后避碰回位

### Description

实现四种 group 共用的 teach 命令。用户进入跟随、第一次回车开始记录、第二次回车冻结并保存；随后恢复保持增益，用已有连接规划返回启动上肢姿态，最后复用同一会话交回。

### Demo path

通过实际 teach CLI 与受控的真实格式反馈事件，分别录制 left_arm、right_arm、arms、arms_waist。第二次回车后手动更新反馈，已冻结目标仍不变；保存文件可由 load_action 原样读取，正常返回全部 upper_q17，回位无解仍保留已录文件。

### 实施合同

- `teach --interface <网卡> --group <四组之一> --config <八增益> --output <动作> --execute`；transition-seconds 默认2且正有限。不增加 side、stage、手部或旧格式；不要求报告/资格文件。没有execute时退出2且未连接。
- `run_teach(interface,group,gains,output,transition_seconds,cancel)` 直接编排既有 RobotSession、save_action、ArmModel、connect、check_trajectory、play。只扩展同一会话，不新增示教发送器或恢复执行器；不重写回放票已拥有的发送/关闭逻辑。
- 17维目标顺序左7+右7+腰Yaw/Roll/Pitch3；四组分别取0–6、7–13、0–13、0–16，文件维数7/7/14/17。未选关节恒为本次 home 与各自 hold 增益。所选手臂采用 arms.teach，只有 arms_waist 选中腰采用 waist.teach；保持、跟随 dq=0，J29只为权重。
- 会话扩展 `follow(group)`、`begin_recording()`、`end_recording()->(Action,frozen_upper_q)`、`hold()`。acquire后follow渐变所选teach增益，完成渐变后提示第一次回车；begin_recording不重复传group，下一条真实回调开始记录，不从旧快照补首帧。
- 每条LowState回调在同一条消息内读取所选位置，时间取本次 monotonic_ns 主机接收时刻；首条真实记录归零，之后严格递增。记录同组同步 q，不复制反馈 dq、不用50Hz发送tick反复盖时间、不伪装设备时间。
- 第二次回车的end_recording在同一把锁内停止记录、关闭follow，以最后已录反馈的选中q覆盖home为完整17维固定目标，dq=0；冻结与动作最后样本同源。保存期间保留teach增益，不能提前切hold。锁外构造Action/写文件，不声称撤回已交SDK的在途包。
- 零记录样本先用最近真实反馈冻结，随后报样本不足；一个样本也先冻结再报不足。两者均不造帧、不写伪动作。新动作唯一落盘格式为 group+frames[{t,q}]，没有dq、schema、notes、qualification或诊断数据。
- 严格顺序 `end_recording → save_action → hold → connect(end_q,home,{group}) → check_trajectory → play → close`。save独占创建，已有文件/保存失败不启动回位；hold才请求恢复两组hold增益。返回继续用动态腰真实模型、活动范围与同一 PPoly。重合端点无连接也正常交回。
- 示教期间主线程保存/规划时发送者继续固定冻结目标；取消和EOF立即复用既有RELEASE，不等待规划、不额外回家。规划取消不再提交结果；录制中途取消不另存partial格式。回位无解或曲线无效发生于保存之后，保留文件并报错误后统一交回。
- Linux输入可取消且可join；只接受相应提示之后的Enter，不让旧Enter跳过录制阶段，EOF始终有效。正常0、用法2、取消130、记录/保存/规划/SDK/清理失败1。所有回放票生命周期合同同样适用，不加入旧阈值或资格门槛。

### Acceptance Criteria

- [ ] 实际 teach CLI 对四组均完整走过接管→follow→两次Enter→保存→hold→真实规划回位→交回；动作各为7/7/14/17维，所选同一帧来自同条反馈，未选关节所有目标均为home，正常结束上肢17维返回启动值。
- [ ] 发送频率与反馈频率故意不同，记录数和时间仍对应真实反馈事件；首帧只从begin_recording后的真实回调产生。文件只含精简字段，实际用已有load_action及action_curve原样读取和构造17维目标，无需转换；安装后的三命令贯通由05验证。
- [ ] 在第二次Enter与保存期间注入新反馈/延迟，冻结目标等于记录末帧展开后的17维而不会继续跟随；保存完成前保留teach增益，只有hold请求后恢复hold。四组的arms/waist增益分配按实际报文核对。
- [ ] 零样本使用最近真实反馈冻结，一个样本冻结其末帧，均返回不足错误且无补造文件；输出已存在时保留旧内容，保存失败无回位play，均通过唯一close交回。
- [ ] 给定回位无精确解或返回曲线无效的真实模型夹具，已录文件仍可读取，程序返回1并统一交回；取消发生于follow/record/save后/返回规划或play时不额外启动home运动，不再提交旧结果，不写partial文件。
- [ ] 提示前旧Enter不会开始或结束当前阶段，EOF可以取消等待；发送与输入线程均join，正常结束不被改成取消。测试保留真实文件、SciPy、MuJoCo/OMPL，替换范围仅SDK通道、时钟、终端事件及精确故障注入边界。
- [ ] 本票自带上述四组示教、同步/冻结/增益/失败路径测试及teach使用说明，不留示教生命周期给最终整合票补齐，不执行机器人动作。

### Blocked by

计划 03。

### Priority

high

### SPEC Reference

§3 分组/文件/增益；§4 示教和RobotSession接口；§5.2–5.3 返回规划；§5.5 完整示教时序；§6 冻结/输入/取消；§7 反馈和报文；§8 错误；§9 T05及对应T01/T02。

---

## 05 · 在干净安装中贯通四组录制产物、混合预览与回放，并交付完整 CI

### Description

把已完成的功能作为可安装、可复现的一套工具交付。最终验证必须从实际teach产物出发，使用同一文件完成preview和replay，不能用分别手写的理想动作冒充贯通。建立Linux完整依赖CI及最小使用说明，覆盖安装资源和完整退出流程。

### Demo path

在干净Linux环境安装wheel并离开项目目录；通过实际CLI离线录出四种group，用生成的arms_waist与left_arm文件直接预览并连续回放，观察返回home17和零权重发送完成；重复取消与回位无解分支。全过程使用真实规划依赖和官方报文，通信构造被替换且禁止联网。

### 实施合同

- 前置功能及各自测试应已完成，本票只实现跨命令产物贯通、干净安装/CI和最终文档，不接收未完成的模型、规划、会话或示教功能，也不把已有单元测试重跑当作新的端到端验收。复用前置故障夹具，不另造覆盖每个阶段的第二套注入矩阵。
- 三命令同一main入口：teach支持left_arm/right_arm/arms/arms_waist；replay接受单/多动作；preview只需要动作与start。文件仅group+frames[t,q]，组维数7/7/14/17，StartPose是leg_q12+upper_q17，八增益只供teach/replay；实际轨迹统一左7+右7+腰3，J29仅权重。
- 端到端测试调用真实CLI分发，实际反馈回调和两次Enter产生四份文件，直接交给preview与replay。动态腰→单臂切换必须使右臂/腰回home；所有进入/过渡/返回使用同一connect，最终曲线检测保留。
- 测试用真实Action读写、SciPy、MuJoCo、OMPL及固定SDK消息/CRC/serialize；仅替换通道/Factory以禁止创建网络、时钟与终端事件，不用整套run_teach、run_replay、plan_sequence替身，不增加生产fake开关或环境模式。headless验证真实viewer绘制数据；实际窗口目视检查与无显示结果分开记录。
- 固定SDK提交为814556d15970dd2ecf1c9984e845ca02ab07e206；完整Linux环境安装其CycloneDDS/IDL依赖和匹配架构的CRC库，SDK安装方式保留原生库可定位路径。离线指不创建网络实体，不能通过不安装依赖然后skip来宣称报文验证通过。
- CI保持两层：普通数值/文件/安装检查；Linux实际规划+官方SDK完整流程。完整层执行T01–T09所含测试，依赖缺失/模型缺失/相关skip不得视为通过，命令失败阻断CI。Windows只承担文件/数值范围，不增加第二套设备实现。
- wheel包含新包与固定模型/网格/来源许可，不包含测试、现场数据、备份、旧脚本或第三方SDK源码。干净安装后从项目目录外读取真实产物、加载模型并完成命令流程；不靠源码工作目录补资源。
- 交付精简README/贡献说明、四种新动作示例、启动姿态示例和唯一八增益配置。清楚写出命令实际参数、正常返回与取消交回的区别；实际增益、动态平衡、固定末端几何和设备跟踪行为属于现场事实，不把离线测试当设备验收，也不新增资格字段/审批/阈值。历史说明保留时标明不适用新版，不保留旧运行脚本或旧命令教程作为现行入口。

### Acceptance Criteria

- [ ] 新增跨命令测试从四次实际teach CLI调用的回调事件生成四份新格式文件，原样输入已安装命令；arms_waist→left_arm的preview绘制目标与replay发送目标来自同一规划合同且完整返回home17。若使用预制动作代替录制产物、整体mock规划/编排或依赖源码目录即失败。
- [ ] 同一端到端环境验证正常0、用户取消130及保存后回位无解1：正常一次接管/返回/交回；取消无额外home；回位失败保留已录文件；通道/线程均回收。该证据贯通真实命令和产物，区别于前置票的局部故障测试。
- [ ] 干净Linux安装的wheel在项目目录外完成帮助、四组文件读取、真实模型载入和上述headless流程；模型及许可证齐全，包中无旧代码、旧工具、数据/备份或厂商SDK。
- [ ] Linux完整CI实际运行T01–T09及上述安装产物流程，规划/官方消息相关依赖缺失或skip导致该层失败；SDK真实CRC/serialize已执行且网络构造次数为0。普通数值/文件层也能独立运行并报告其覆盖范围。
- [ ] 新交付的README和示例在干净安装中实际执行四组录制及混合序列，包含三命令、start和八增益所需的全部输入，动作文件无额外字段；不存在缺参数或引用旧格式/手部/资格报告才能执行的现行用法。最终帮助维持三命令，业务模块维持八个，旧阈值/settle不回流。
- [ ] 用可复制命令记录安装/CI结果；图形窗口目视结果仅在实际完成时声明，代码/几何/报文通过与设备验收分开说明，不执行真机动作。

### Blocked by

计划 04。

### Priority

high

### SPEC Reference

§2 最终模块与依赖；§3–§4 最终接口；§5.5–5.6 跨命令流程；§6–§8 收尾/报文/退出；§9 T09、T01–T09完整CI；§10 资源打包与交付；§11 离线证据边界。

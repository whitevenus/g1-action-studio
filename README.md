# G1 Action Studio

G1 Arm7 上肢动作工具，支持左臂、右臂、双臂、双臂加腰四种范围。
当前版本实现四组动作的离线预览及受阻绕行（#9–#10）；设备回放和示教在 #11–#12 实现。

```sh
uv sync --extra planning
uv run g1-action-studio preview examples/arms_waist.json examples/left_arm.json --start examples/start.json
uv run g1-action-studio preview examples/right_arm.json --start examples/start.json --speed 0.5 --transition-seconds 2
```

图形预览以 Linux 为交付平台。查看器显示进入、动作、衔接和返回启动姿态，关闭窗口退出，Ctrl+C 取消。
四组示例每个动作持续8秒，包含明显抬臂、侧展和屈肘；双臂加腰另有左右转腰及小幅俯仰、侧倾。
肩 Pitch 最大约120°、侧展约86°，腰左右转动各约57°。默认单动作共12秒，上述双动作组合共22秒；只播放一遍，结束后保持启动姿态。
连接先尝试直连，受阻时用 OMPL RRTConnect 在所选动作组内绕行；没有精确解就报错。
动作本身碰撞或曲线越界仍会直接报错。预览只做 FK 显示，不初始化 SDK 或 DDS。

左臂绕行示例：从启动姿态直连目标会碰到左腿，程序规划绕行，保持目标4秒后绕行返回；右臂与腰固定。

```sh
uv run --extra planning g1-action-studio preview examples/detour_left_arm.json --start examples/start.json --transition-seconds 4
```

进入与返回各4秒，总共12秒。绕行路径可能随搜索变化；折线拐点允许停顿。
单臂只搜索7维，左右换组开放双臂14维，含腰换组开放17维；搜索最多5秒，随后最多1秒删点，连接总时长仍由 `--transition-seconds` 决定。
仅连接段会绕行，不能通过重规划改写原动作。

动作只有 `group` 与 `frames`，每帧只有 `t`、`q`：

```json
{
  "group": "left_arm",
  "frames": [
    {"t": 0, "q": [0, 0.2, 0, 0.8, 0, 0, 0]},
    {"t": 1, "q": [0.1, 0.2, 0, 0.9, 0, 0, 0]}
  ]
}
```

| group | q 的关节顺序 | 维数 |
| --- | --- | --- |
| left_arm | 左肩 Pitch/Roll/Yaw、肘、腕 Roll/Pitch/Yaw | 7 |
| right_arm | 右侧同序 | 7 |
| arms | 左臂、右臂 | 14 |
| arms_waist | 左臂、右臂、腰 Yaw/Roll/Pitch | 17 |

角度 rad，时间秒，首帧 t=0，至少两帧且时间递增。起点文件只有 `leg_q[12]` 和 `upper_q[17]`；腿固定，上肢顺序为左7、右7、腰3。
未选关节保持起点。文件不存 dq；同一条轨迹同时产生 q/dq。`--speed` 只缩放动作时间和速度，`--transition-seconds` 决定每个非零连接段总时长。

只接受新格式，不兼容旧 side、v1/v2、profile 或旧命令。当前没有可动手指数据或控制。
随包模型来自固定版本的官方 G1 29DoF；固定末端及碰撞覆盖见 [模型来源](src/g1_action_studio/resources/ORIGIN.md)。
几何检测不求解动态平衡或未知环境，不表示实机动作验收。示例数字仅用于说明和离线流程。

开发与测试见 [CONTRIBUTING](CONTRIBUTING.md)，重写合同见 [PRD](tasks/prd-project-rebuild.md) 和 [SPEC](tasks/spec-project-rebuild.md)。
既有 docs/tools 中的旧流程属于历史资料，不适用于新版。

# G1 Action Studio

G1 Arm7 上肢动作工具，支持左臂、右臂、双臂、双臂加腰四种范围。
当前版本实现离线直连预览（#9）；受阻绕行、设备回放和示教依次在 #10–#12 实现。

```sh
uv sync --extra planning
uv run g1-action-studio preview examples/arms_waist.json examples/left_arm.json --start examples/start.json
uv run g1-action-studio preview examples/right_arm.json --start examples/start.json --speed 0.5 --transition-seconds 2
```

图形预览以 Linux 为交付平台。查看器显示进入、动作、衔接和返回启动姿态，关闭窗口退出，Ctrl+C 取消。
直连碰撞或曲线越界会直接报错；当前版本不会绕行，也不改写动作。预览只做 FK 显示，不初始化 SDK 或 DDS。

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

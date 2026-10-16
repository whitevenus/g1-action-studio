# 开发说明

Python 3.11，标准 src 包布局，unittest 与 Ruff。依赖继续用 uv 管理：

```sh
uv sync --extra dev --extra planning
uv run python -m unittest discover -s tests -v
uv tool run ruff==0.16.8 check src tests
uv tool run ruff==0.16.8 format --check src tests
uv run python -m build
```

普通文件/数值测试不需要 MuJoCo；模型、规划和预览测试需要 planning 依赖，缺失依赖应失败，不用 skip 代替验证。
图形预览以 Linux 为交付平台，Windows 可执行数值、文件及无窗口模型测试。测试不连接机器人，headless 验证绘制数据而非窗口目视效果。

当前仅 preview 可用。新代码按 PRD/SPEC 从空包写，不恢复旧格式、旧入口、灵巧手或旧回放阈值。
业务模块保持直接调用：motion 管文件与 PPoly，model 管唯一关节映射和 MuJoCo，planner 管完整曲线与连接，preview 显示同一曲线，cli 解析参数。
不添加转发包装、同值别名、插件框架或临时模式。后续 robot/teach/replay 合入前不创建占位模块。

关节映射集中于 model。model 顶层不加载 MuJoCo；planner 不依赖设备；曲线碰撞检测只在规划阶段进行。
模型与 viewer 各持自己的 MjData；viewer 使用 FK，不调用 mj_step 生成另一条运动。

项目提示与说明使用中文，标识符与第三方许可保持原文。Python 使用双引号、四空格、120字符行宽和 CRLF。
验证 wheel 时必须从项目目录外加载包和模型资源，不能依赖源码工作目录。不要提交个人录制、凭据、重写备份或厂商SDK。

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

当前提供 preview、replay、teach。新代码按 PRD/SPEC 从空包写，不恢复旧格式、旧入口、灵巧手或旧回放阈值。
业务模块保持直接调用：motion 管文件与 PPoly，model 管唯一关节映射和 MuJoCo，planner 管完整曲线与连接，preview 显示同一曲线，robot 管 SDK 和唯一发送循环，replay 与 teach 编排会话，cli 管配置与终端输入。
不添加转发包装、同值别名、插件框架或临时模式。

Linux 绕行、无解和取消用例可单独运行：

```sh
uv run python -m unittest discover -s tests -p 'test_rrt.py' -v
```

成功用例使用真实模型和 RRTConnect，覆盖7/14/17维、最终曲线及固定的非活动关节。
无解用例将左臂除肩 Yaw 外的搜索范围收窄到启动值附近，让真实模型的碰撞带隔开起终点；真实搜索得到的近似解必须拒绝。
收窄范围仅用于构造确定无解的测试，生产搜索始终采用模型 XML 全部活动关节的原始范围。另验证搜索及快捷删点期间取消不会返回轨迹。

回放与示教正式测试使用官方 SDK `814556d15970dd2ecf1c9984e845ca02ab07e206`、CycloneDDS 0.10.2 和真实 CRC 库。Linux 安装示例：

```sh
uv sync --locked --extra dev --extra planning
git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/g1-cyclone-src
cmake -S /tmp/g1-cyclone-src -B /tmp/g1-cyclone-build -DCMAKE_INSTALL_PREFIX=/tmp/g1-cyclone -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_DDSPERF=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
cmake --build /tmp/g1-cyclone-build --parallel 4
cmake --install /tmp/g1-cyclone-build
CYCLONEDDS_HOME=/tmp/g1-cyclone uv pip install cyclonedds==0.10.2
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/g1-unitree-sdk
git -C /tmp/g1-unitree-sdk checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
uv pip install --no-deps -e /tmp/g1-unitree-sdk
uv run python -m unittest discover -s tests -v
```

SDK 以 editable 安装，保留随附本架构 CRC `.so` 的路径；Python 3.11 的 CycloneDDS 0.10.2 需要上述 native 构建。
若项目环境通过 `UV_PROJECT_ENVIRONMENT` 放在其他目录，两个 `uv pip install` 也需显式添加 `--python "$UV_PROJECT_ENVIRONMENT/bin/python"`。之后再次同步依赖时使用 `uv sync --inexact` 保留单独安装的 SDK/CycloneDDS。
普通文件、配置和数值测试可单独运行 `test_motion.py`、`test_cli.py`；SDK 报文、完整回放、示教及生命周期测试在 Linux 安装上述依赖后全部运行，缺失依赖应失败。
测试替换 Factory 和通道构造，禁止创建网络实体；官方消息、CRC、序列化、SciPy、MuJoCo 与 OMPL 均执行真实实现。
示教测试用真实管道终端驱动两次回车，核对四组反馈采样、增益切换、冻结保存、真实返回规划，以及 EOF、取消、保存和规划失败；发送与输入线程均须 join。

关节映射集中于 model。model 顶层不加载 MuJoCo；planner 不依赖设备；曲线碰撞检测只在规划阶段进行。
模型与 viewer 各持自己的 MjData；viewer 使用 FK，不调用 mj_step 生成另一条运动。

项目提示与说明使用中文，标识符与第三方许可保持原文。Python 使用双引号、四空格、120字符行宽和 CRLF。
验证 wheel 时必须从项目目录外加载包和模型资源，不能依赖源码工作目录。不要提交个人录制、凭据、重写备份或厂商SDK。

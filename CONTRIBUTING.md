# 开发说明

Python 3.11、unittest、Ruff，标准 src 包布局，依赖由 uv.lock 固定。

普通文件/数值层不需要 MuJoCo 或 SDK，Linux 与 Windows 均可运行：

```sh
uv sync --locked --extra dev
uv run --no-sync python tests/run_tests.py numerical
uv tool run ruff==0.16.8 check src tests
uv tool run ruff==0.16.8 format --check src tests
uv build --wheel
uv run --no-sync python tests/verify_wheel.py dist/g1_action_studio-0.1.0-py3-none-any.whl
```

数值层覆盖文件、配置、曲线、CLI 用法与 wheel 内容；Linux 另测终端输入，Windows 跳过 Linux 终端用例。它不代表规划、官方报文或完整流程通过。

Linux 完整层先安装规划依赖、固定官方 SDK 与 CycloneDDS 0.10.2。SDK 提交为 `814556d15970dd2ecf1c9984e845ca02ab07e206`，使用 editable 安装保留本架构 CRC `.so` 的定位路径：

```sh
uv sync --locked --extra dev --extra planning
project_python="$(uv run --no-sync python -c 'import sys; print(sys.executable)')"
git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/g1-cyclone-src
cmake -S /tmp/g1-cyclone-src -B /tmp/g1-cyclone-build -DCMAKE_INSTALL_PREFIX=/tmp/g1-cyclone -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_DDSPERF=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
cmake --build /tmp/g1-cyclone-build --parallel 4
cmake --install /tmp/g1-cyclone-build
CYCLONEDDS_HOME=/tmp/g1-cyclone uv pip install --python "$project_python" cyclonedds==0.10.2
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/g1-unitree-sdk
git -C /tmp/g1-unitree-sdk checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
uv pip install --python "$project_python" --no-deps -e /tmp/g1-unitree-sdk
uv run --no-sync python tests/run_tests.py full
```

`run_tests.py full` 执行 SPEC T01–T09；依赖缺失、测试失败或任何 skip 都返回非零，不能用跳过代替验证。SDK 单独安装后使用 `uv run --no-sync`；再次同步依赖可用 `uv sync --inexact --locked --extra dev --extra planning` 保留 SDK。`project_python` 也适用于通过 `UV_PROJECT_ENVIRONMENT` 放在项目外的环境。

干净安装的完整贯通（仍在 Linux，沿用上面构建的 CycloneDDS 与固定 SDK）：

```sh
project_dir="$PWD"
uv build --wheel
uv export --locked --extra planning --no-emit-project --no-hashes --output-file /tmp/g1-planning.txt
uv venv --python 3.11 /tmp/g1-installed
uv pip install --python /tmp/g1-installed/bin/python dist/g1_action_studio-0.1.0-py3-none-any.whl -r /tmp/g1-planning.txt
CYCLONEDDS_HOME=/tmp/g1-cyclone uv pip install --python /tmp/g1-installed/bin/python cyclonedds==0.10.2
uv pip install --python /tmp/g1-installed/bin/python --no-deps -e /tmp/g1-unitree-sdk
cd /tmp
/tmp/g1-installed/bin/g1-action-studio --help
/tmp/g1-installed/bin/python -m g1_action_studio --help
/tmp/g1-installed/bin/python "$project_dir/tests/verify_wheel.py" "$project_dir/dist/g1_action_studio-0.1.0-py3-none-any.whl" --installed
/tmp/g1-installed/bin/python "$project_dir/tests/verify_install.py" "$project_dir/examples"
/tmp/g1-installed/bin/python "$project_dir/tests/run_tests.py" workflow
```

贯通测试通过四次实际 teach CLI 和两次回车生成四种动作文件，原样输入 preview/replay；验证动态腰切换到左臂、统一 q/dq、home17、零权重、取消、保存后回位无精确解及资源回收。文件临时独立创建和清理，不用手写理想动作代替录制。无解复用真实模型的窄搜索盒夹具，生产始终采用 XML 原始搜索范围。

测试保留 Action 读写、SciPy、MuJoCo、OMPL、官方消息、CRC 和序列化，替换 SDK Factory/通道、时钟、终端事件与 headless viewer；禁止创建 DDS 网络实体。headless 检查真实 MjData 绘制目标，不能宣称完成图形窗口目视或真机验收。实际增益、动态平衡、固定末端几何对应关系及设备跟踪行为需现场事实，本轮不执行机器人动作。

GitHub CI 的 numerical 层独立运行数值/文件和安装检查；preview 层按锁文件准备完整 Linux 依赖、运行零 skip 的全套测试，再从项目外安装 wheel 跑真实录制产物贯通。wheel 内容白名单与资产指纹检查确认八个业务模块、模型、网格、来源和许可齐全，不包含测试、数据、备份、旧工具或厂商 SDK。

业务模块保持直接调用：motion 管文件与 PPoly，model 管唯一映射和几何，planner 管曲线与连接，preview 显示同一曲线，robot 管 SDK 与唯一发送循环，teach/replay 编排会话，cli 管配置与输入。模型与 viewer 各持 MjData；viewer 使用 FK，不调用 mj_step 生成另一条运动。

项目提示与说明使用中文，标识符与第三方许可保持原文；Python 使用双引号、四空格、120 字符行宽和 CRLF。不恢复旧格式、旧入口、灵巧手、阈值或 settle，不添加转发包装、临时模式或第二套执行器。不要提交个人录制、凭据、重写备份或第三方 SDK。旧文档及图示见 [历史资料](docs/README.md)。

# 开发说明

源码直接在 `src/` 中修改。先在项目根目录安装，再运行测试：

```sh
python -m pip install -e '.[dev]'
python -m unittest discover -s tests -v
g1-action-studio --help
python -m build
```

项目说明、注释和用户提示使用中文；Python 标识符、命令参数、协议字段和第三方许可保留原名。

基础功能支持 Python 3.8+。避碰测试需要 Python 3.10+ 和 `python -m pip install -e '.[planning]'`。
缺少 MuJoCo 时会跳过避碰测试；跳过不算该功能验证通过。
CI 会单独安装 MuJoCo 运行避碰测试。

默认测试不连接机器人。`tests/fake_sdk.py` 提供内存消息与虚拟时钟，只用于测试，不随项目安装。
不要把真机动作加入默认测试。
验证安装包时应从项目目录之外运行命令，确认资源随包安装。

优先使用直接的函数和已有上下文，避免重复实现、只转发调用的包装层和暂时用不到的框架。
共享常量放在 `constants.py`，轨迹计算放在 `trajectory.py`，参数校验和 JSON 读写放在 `config.py`。
业务模块直接调用这些函数，不跨模块引用带下划线的私有工具函数。
示教与回放必须保留显式 `--execute`、反馈有效性检查、输出不覆盖和异常释放流程。

示例参数和合成轨迹不能视为通用真机参数。记录验证结果时，区分离线检查、SDK 写入、实测反馈和现场动作/交接。
不要提交凭据、私有设备地址、厂商 SDK 二进制或个人录制；回归测试使用合成样本。

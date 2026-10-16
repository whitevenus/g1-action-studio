# Dex3-1 电机下标验证

> 2026-10-16：仅手臂控制已接入，使用 `teach-arm7.json`／`replay-arm7.json`，不等待手部反馈或发送手部命令。Dex3 控制实现保留；现场手部 DDS 未适配的问题仍未解决。本工具继续用于以后核实实际编号。历史证据见 [现场结果与后续方向](dex3-status-and-next-steps-2026-10-15.md)，当前用法见仓库 README。

使用 `verify_dex3_indices.py` 只读订阅灵巧手状态，通过分别活动手指，核实原始 DDS 数组下标与物理手指的对应关系。

官方 DDS 网页与官方遥操作示例的右手定义存在差异，本流程用于确认设备实际反馈：

| 原始 DDS 下标 | 官方遥操作示例：左手 | 官方遥操作示例：右手 | 官方 DDS 网页表格 |
| --- | --- | --- | --- |
| 0、1、2 | 拇指 | 拇指 | 拇指 |
| 3、4 | 中指 | 食指 | 中指 |
| 5、6 | 食指 | 中指 | 食指 |

参考资料：

- [官方 DDS 通信文档](https://support.unitree.com/home/zh/dex3-1_hand/DDS-based_communication_method)
- [官方 IDL 字段说明](https://support.unitree.com/home/zh/dex3-1_hand/Dex_IDL_specification)
- [官方遥操作示例](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_hand_unitree.py)

## 运行前提

- 机器人侧使用已安装 `unitree_sdk2py` 及其 DDS 依赖的 Python 环境。
- 机器人手部状态服务正在发布状态；脚本不会启动或切换服务。
- 脚本只订阅 `rt/dex3/{left|right}/state`，不创建命令发布者，不使电机卸力。
- 仅在现场允许手动活动手指的条件下操作。若有保持力，不要强掰。

## 1. 复制脚本

在本机项目根目录的 PowerShell 中执行，替换用户名和机器人地址：

```powershell
scp .\tools\verify_dex3_indices.py 用户名@机器人地址:~/
```

后续命令在机器人终端中执行。

## 2. 确认 Python 环境与网卡

```bash
python3 -c "from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_; print('SDK imports OK')"
ip -br addr
```

若 SDK 导入失败，先切换至已配置 SDK 的 Python 环境。以下示例假设 DDS 网卡为 `eth0`，domain 为 `0`；请按设备实际配置替换。可用 `--domain` 显式指定 domain。

## 3. 测试右手中指

```bash
python3 ~/verify_dex3_indices.py \
  --interface eth0 \
  --domain 0 \
  --side right \
  --finger middle \
  --seconds 15 \
  --output right-middle.json
```

1. 收到状态后，前 3 秒保持手指静止。
2. 看到“开始”提示后，只轻轻活动右手中指，往返几次，尽量让两个中指关节都有变化。
3. 其他手指尽量保持静止。
4. 等待记录结束，查看各槽位的变化范围。

## 4. 测试右手食指

```bash
python3 ~/verify_dex3_indices.py \
  --interface eth0 \
  --domain 0 \
  --side right \
  --finger index \
  --seconds 15 \
  --output right-index.json
```

操作与上一步相同，但本次只活动右手食指。

`--finger` 仅是操作者给本次报告添加的标签，不控制手指，也不改变数据顺序。

## 5. 判断映射

终端的 `q` 始终按原始 `motor_state[0..6]` 排列，单位为 rad。结束表中的变化范围为记录期间的最大角度减去最小角度。

| 实际活动手指 | 若官方遥操作示例适用 | 若官方 DDS 网页表格适用 |
| --- | --- | --- |
| 右手中指 | 下标 5、6 变化明显 | 下标 3、4 变化明显 |
| 右手食指 | 下标 3、4 变化明显 | 下标 5、6 变化明显 |

比较两次报告，观察变化是否稳定地对应指定手指。不要仅凭一次变化最大的槽位作结论；若变化太小或多个手指联动，应重测。

若中指对应 5、6，食指对应 3、4，则项目当前右手映射 `(0, 1, 2, 5, 6, 3, 4)` 与该设备反馈一致，它把原始 DDS 顺序整理为“拇指、中指、食指”。

验证左手时，将 `--side` 改为 `left`，并使用不同报告文件名。上述两份资料对左手均为 3、4 对应中指，5、6 对应食指。

## 6. 保存和取回结果

JSON 报告保存在运行命令时的当前目录，包含话题、网卡、domain、人工手指标签、有效样本数、各槽位最小角度、最大角度和变化范围。

在本机 PowerShell 中取回报告：

```powershell
scp 用户名@机器人地址:~/right-middle.json .\tools\
scp 用户名@机器人地址:~/right-index.json .\tools\
```

上述取回命令假设运行脚本时位于机器人用户的主目录；若不是，请替换为报告实际路径。报告不覆盖已有文件，重测时请使用新的 `--output` 文件名。

## 异常处理与验证范围

- 5 秒未收到有效状态：检查网卡、domain、SDK 环境与手部状态服务。
- 状态超过 0.5 秒未更新，或出现长度错误、非有限角度：脚本退出，本次不生成完整报告。
- 按 `Ctrl+C` 可中止并关闭订阅；本次不生成完整报告。
- 本流程只核实反馈槽位与物理手指的对应关系，不验证命令执行、控制权、卸力或控制交接。

本地已完成合成消息的离线验证；机器人侧的实际 DDS 接收和手指对应关系需要按本流程实测。

`tools/` 不再被整体忽略，脚本和说明可以纳入 Git，供其他电脑继续工作。个人录制、现场原始报告及含设备地址的终端记录请保存在被忽略的 `data/` 目录中；不要直接提交。可将 `--output` 指向该目录下的新文件。

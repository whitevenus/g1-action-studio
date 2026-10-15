# 第三方来源与许可

本项目原创代码采用 MIT 许可，完整文本见 `LICENSE`。第三方许可原文和版权声明保持原样。

## 关节范围

`src/resources/g1_arm7_limits.xml` 仅提取了 `unitreerobotics/unitree_mujoco` 中
`unitree_robots/g1/g1_29dof.xml` 的 14 个手臂关节名称与范围。
原始来源采用 BSD-3-Clause 许可，版权声明和完整许可保存在 `LICENSE-UNITREE`。
这份文件不是完整 MuJoCo 模型，也不是已验收的真机安全范围。

## 碰撞模型

`src/resources/collision/` 来自[宇树 xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
的 `assets/g1/g1_body29_hand14.xml` 及其 STL 资源，对应 G1 29DoF + Dex3-1。
版权所有：2025 HangZhou YuShu TECHNOLOGY CO., LTD.
原始声明保存在 `LICENSE-UNITREE-MODEL`，Apache-2.0 完整许可见 `LICENSE-APACHE-2.0`。

本项目移除了执行器、自由基座、重复的可视/仿真几何和手指关节；碰撞几何使用凸网格及保守的手指扫掠球。
基座固定，29 个机体/手臂关节用于正向运动学。
来源、资源哈希及手指包络半径保存在 `src/resources/collision/provenance.json`。

## 真机 SDK

项目不附带厂商二进制、SDK 实现或真实设备录制。
真机功能使用单独安装的[宇树 SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)，
其源码遵循自身许可。本项目独立开发，不代表宇树官方认可。

## 规划库

路径搜索和路点简化使用 [OMPL](https://github.com/ompl/ompl) 2.0.1（BSD-3-Clause）；FK 与几何距离查询使用 [MuJoCo](https://github.com/google-deepmind/mujoco)（Apache-2.0）。两者由 `planning` 依赖安装，项目不复制其算法实现。

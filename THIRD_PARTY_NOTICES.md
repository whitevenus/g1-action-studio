# 第三方来源与许可

本项目原创代码采用 MIT 许可，完整文本见 `LICENSE`。第三方许可原文和版权声明保持原样。

## 碰撞模型

`src/resources/collision/` 来自[宇树 xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
的 `assets/g1/g1_body29_hand14.xml` 及其 STL 资源，对应 G1 29DoF + Dex3-1。
版权所有：2025 HangZhou YuShu TECHNOLOGY CO., LTD.
原始声明保存在 `LICENSE-UNITREE-MODEL`，Apache-2.0 完整许可见 `LICENSE-APACHE-2.0`。

本项目固定基座，移除了执行器、传感器、显式惯性和重复几何；保留29个腿腰/手臂关节，恢复14个官方手指关节。
手指使用官方网格和碰撞几何；拇指中段官方碰撞box与可视网格分开，未添加手指球形包络。
来源与49份STL哈希保存在 `src/resources/collision/provenance.json`。

## 真机 SDK

项目不附带厂商二进制、SDK 实现或真实设备录制。
真机功能使用单独安装的[宇树 SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)，
其源码遵循自身许可。本项目独立开发，不代表宇树官方认可。

## 规划库

路径搜索和路点简化使用 [OMPL](https://github.com/ompl/ompl) 2.0.1（BSD-3-Clause）；FK 与几何距离查询使用 [MuJoCo](https://github.com/google-deepmind/mujoco)（Apache-2.0）。两者由 `planning` 依赖安装，项目不复制其算法实现。

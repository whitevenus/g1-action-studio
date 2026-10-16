# 第三方来源与许可

项目原创代码使用 MIT，见 LICENSE。第三方许可原文和版权声明保留。

随包 G1 29DoF 模型来自 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)，固定提交 ab03eec53238487c4a7c7f61dd8e29a2f66abc04，采用其 BSD-3-Clause 许可。
XML、网格清单、实际碰撞覆盖及许可见 [ORIGIN](src/g1_action_studio/resources/ORIGIN.md) 和同目录 LICENSE。
旧 Dex3 模型已移除，原根目录第三方许可保留用于历史来源追溯。

数值曲线使用 NumPy（BSD-3-Clause）和 SciPy（BSD-3-Clause）；FK/几何使用 MuJoCo（Apache-2.0）。
后续路径搜索依赖 OMPL 2.0.1（BSD-3-Clause）；#9 不调用它。库由依赖管理安装，本项目不复制其算法。
项目不附带厂商 SDK、二进制或真实设备数据；独立开发，不代表厂商认可。

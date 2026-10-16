# 官方 G1 29DoF 模型

- 上游仓库：https://github.com/unitreerobotics/unitree_mujoco
- 固定提交：`ab03eec53238487c4a7c7f61dd8e29a2f66abc04`（PRD/SPEC 中的 `ab03eec`）。
- XML：`unitree_robots/g1/g1_29dof.xml`，原样保留。
- 网格：只包含该 XML 直接引用的 36 个 `unitree_robots/g1/meshes/*.STL`，原样保留。
- 许可证：上游根目录 BSD-3-Clause，原文随此目录的 `LICENSE` 分发。
- 导入核对：XML、36 个网格及 LICENSE 共 38 个文件的 Git blob SHA-1 均与固定提交树一致。XML 的 SHA-256 为 `423e28bd718b19f7a65cda539b6f794ddbb268b4b9bdbd85f4bd982b30729617`。

模型的自由基座保持上游初始位置与单位四元数。十二个腿部关节固定为启动姿态，双臂十四关节及腰部三关节参与前向运动学。碰撞使用原 XML 的碰撞几何、掩码和 MuJoCo 装配过滤规则；纯显示副本不参与碰撞。

XML 中的 `left_rubber_hand`、`right_rubber_hand` 是固定末端显示网格，既没有可动手指关节，也没有手部控制通道。其 `contype=0`、`conaffinity=0` 保持不变，因此不代表固定末端外形已经参与碰撞检测。该资源不包含 Dex3 模型。

离线加载与轮包安装不联网下载模型。有限分辨率的几何检测不代表动态平衡或实际设备验收。

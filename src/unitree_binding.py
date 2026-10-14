"""真实 Unitree SDK 接口绑定；默认禁用网络，导入和构造消息不初始化 DDS。"""

from .preflight import _read_arm_service, _read_control_state


class UnitreeReplaySDK:
    def __init__(self, *, offline_transport=None, service_clients=None):
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
        from unitree_sdk2py.core import channel

        # 类型交给 Channel，默认工厂只负责构造带 35 个电机槽位的实例。
        self.LowCmdType, self.LowStateType = LowCmd_, LowState_
        self.LowCmd, self.LowState = unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
        self.CRC = CRC
        self.channel = channel
        self.offline_transport = offline_transport
        self.service_clients = service_clients
        self.network_initialized = False

    def initialize_network(self, interface, domain=0, *, allow_network=False):
        """真机回放的显式网络边界；离线入口不会调用此方法。"""
        if not allow_network:
            raise RuntimeError("网络初始化未授权；默认仅允许离线报文验证")
        if self.offline_transport is not None or self.network_initialized:
            raise RuntimeError("离线绑定或已初始化绑定不能重新初始化网络")
        if not isinstance(interface, str) or not interface.strip() or type(domain) is not int or domain < 0:
            raise ValueError("DDS 网卡或域无效")
        self.channel.ChannelFactoryInitialize(domain, interface)
        self.network_initialized = True

    def _channels(self):
        if self.offline_transport is not None:
            return self.offline_transport
        if not self.network_initialized:
            raise RuntimeError("DDS 未初始化；拒绝创建真实通道")
        return self.channel

    def ChannelSubscriber(self, topic, message_type):
        return self._channels().ChannelSubscriber(topic, message_type)

    def ChannelPublisher(self, topic, message_type):
        return self._channels().ChannelPublisher(topic, message_type)

    def initialize_service_clients(self):
        if not self.network_initialized or self.offline_transport is not None:
            raise RuntimeError("离线模式禁止创建真实 RPC 客户端")
        if self.service_clients is not None:
            raise RuntimeError("RPC 客户端已提供")
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        # 这些客户端的构造函数本身会创建通信通道，不能在离线加载时构造。
        loco, switcher, robot_state = LocoClient(), MotionSwitcherClient(), RobotStateClient()
        for client in (loco, switcher, robot_state):
            client.SetTimeout(2.0)
            client.Init()
        self.service_clients = (loco, switcher, robot_state)

    def read_services(self):
        if self.service_clients is None:
            raise RuntimeError("未提供服务查询客户端；离线验证须显式注入替身")
        loco, switcher, robot_state = self.service_clients
        return _read_control_state(loco, switcher), _read_arm_service(robot_state)

"""真实 Unitree SDK 接口绑定；默认禁用网络，导入和构造消息不初始化 DDS。"""

import time


def read_control_state(loco, switcher):
    for attempt in range(3):
        code, mode = switcher.CheckMode()
        # SDK 的 3104 表示等待 RPC 回复超时；只重试只读查询。
        if code != 3104 or attempt == 2:
            break
        print("CheckMode 查询超时，稍后进行第 {} 次只读查询。".format(attempt + 2), flush=True)
        time.sleep(0.5)
    if code != 0 or not isinstance(mode, dict) or not isinstance(mode.get("name"), str):
        raise RuntimeError("CheckMode 失败：返回码={!r}，数据={!r}".format(code, mode))
    code, fsm_id = loco.GetFsmId()
    if code != 0 or type(fsm_id) is not int:
        raise RuntimeError("GetFsmId 失败：返回码={!r}，数据={!r}".format(code, fsm_id))
    return {"motion_service": mode["name"], "fsm_id": fsm_id}


def read_arm_service(robot_state):
    code, services = robot_state.ServiceList()
    if code != 0 or services is None:
        raise RuntimeError("ServiceList 失败：返回码={!r}".format(code))
    for service in services:
        if service.name == "g1_arm_example":
            return {"name": service.name, "status": service.status, "protect": service.protect}
    return None


def create_service_clients():
    """构造客户端会创建 RPC 通道；仅在网络已初始化后调用。"""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

    clients = []
    for factory in (LocoClient, MotionSwitcherClient, RobotStateClient):
        client = factory()
        client.SetTimeout(2.0)
        client.Init()
        clients.append(client)
    return tuple(clients)


class UnitreeReplaySDK:
    def __init__(self):
        from unitree_sdk2py.core import channel
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__LowCmd_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        # 类型交给 Channel，默认工厂只负责构造带 35 个电机槽位的实例。
        self.LowCmdType, self.LowStateType = LowCmd_, LowState_
        self.LowCmd = unitree_hg_msg_dds__LowCmd_
        self.CRC = CRC
        self.channel = channel
        self.service_clients = None
        self.network_initialized = False

    def initialize_hand_types(self):
        if hasattr(self, "HandCmdType"):
            return
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

        self.HandCmd, self.HandCmdType, self.HandStateType = unitree_hg_msg_dds__HandCmd_, HandCmd_, HandState_

    def initialize_network(self, interface, domain=0, *, allow_network=False):
        """真机回放的显式网络边界；离线入口不会调用此方法。"""
        if not allow_network:
            raise RuntimeError("网络初始化未授权；默认仅允许离线报文验证")
        if self.network_initialized:
            raise RuntimeError("已初始化绑定不能重新初始化网络")
        if not isinstance(interface, str) or not interface.strip() or type(domain) is not int or domain < 0:
            raise ValueError("DDS 网卡或域无效")
        self.channel.ChannelFactoryInitialize(domain, interface)
        self.network_initialized = True

    def _channels(self):
        if not self.network_initialized:
            raise RuntimeError("DDS 未初始化；拒绝创建真实通道")
        return self.channel

    def ChannelSubscriber(self, topic, message_type):
        return self._channels().ChannelSubscriber(topic, message_type)

    def ChannelPublisher(self, topic, message_type):
        return self._channels().ChannelPublisher(topic, message_type)

    def initialize_service_clients(self):
        if not self.network_initialized:
            raise RuntimeError("离线模式禁止创建真实 RPC 客户端")
        if self.service_clients is not None:
            raise RuntimeError("RPC 客户端已提供")
        self.service_clients = create_service_clients()

    def read_services(self):
        if self.service_clients is None:
            raise RuntimeError("未初始化服务查询客户端")
        loco, switcher, robot_state = self.service_clients
        return read_control_state(loco, switcher), read_arm_service(robot_state)


def create_sdk(interface):
    """示教与回放共用的显式真机连接入口；调用前由上层确认 execute。"""
    sdk = UnitreeReplaySDK()
    sdk.initialize_network(interface, allow_network=True)
    sdk.initialize_service_clients()
    return sdk

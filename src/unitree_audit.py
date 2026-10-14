"""真实 SDK 的离线接口、CRC 与序列化审计；通信层全部替换且阻断网络。"""

import hashlib
import importlib
import importlib.metadata
import inspect
import socket
import struct
from contextlib import ExitStack, contextmanager
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch

from . import ARM_SLOTS, WEIGHT_SLOT
from .executor import execute_replay, validate_plan
from .replay_sdk import ReplaySdkPort
from .simulated_sdk import SIMULATION_PROFILE, SimulatedSDK, VirtualClock
from .unitree_binding import UnitreeReplaySDK


@contextmanager
def block_network():
    """任何遗漏的 DDS/RPC/套接字构造都是验证失败，不能变成真实通信。"""
    attempted = []
    def forbidden(*args, **kwargs):
        attempted.append("network_or_dds_call")
        raise RuntimeError("离线审计禁止创建 DDS、RPC 或网络通信")
    with ExitStack() as stack:
        stack.enter_context(patch.object(socket.socket, "__init__", forbidden))
        # 在导入 Unitree SDK 前阻断 Cyclone DDS 实体创建。
        for module_name, class_name in (("cyclonedds.domain", "Domain"),
                                        ("cyclonedds.domain", "DomainParticipant"),
                                        ("cyclonedds.topic", "Topic"),
                                        ("cyclonedds.pub", "Publisher"),
                                        ("cyclonedds.pub", "DataWriter"),
                                        ("cyclonedds.sub", "Subscriber"),
                                        ("cyclonedds.sub", "DataReader")):
            cls = getattr(importlib.import_module(module_name), class_name)
            stack.enter_context(patch.object(cls, "__init__", forbidden))
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        interfaces = {
            "ChannelPublisher": str(inspect.signature(channel.ChannelPublisher)),
            "ChannelSubscriber": str(inspect.signature(channel.ChannelSubscriber)),
            "Publisher.Init": str(inspect.signature(channel.ChannelPublisher.Init)),
            "Publisher.Write": str(inspect.signature(channel.ChannelPublisher.Write)),
            "Subscriber.Init": str(inspect.signature(channel.ChannelSubscriber.Init)),
            "ChannelFactoryInitialize": str(inspect.signature(channel.ChannelFactoryInitialize)),
        }
        # 只绑定参数，不调用接口。SDK 签名变化应使离线审计失败。
        for function, args in (
            (channel.ChannelSubscriber, ("rt/lowstate", object)),
            (channel.ChannelPublisher, ("rt/arm_sdk", object)),
            (channel.ChannelSubscriber.Init, (None, lambda message: None, 10)),
            (channel.ChannelPublisher.Init, (None,)),
            (channel.ChannelPublisher.Write, (None, object())),
            (channel.ChannelSubscriber.Close, (None,)),
            (channel.ChannelPublisher.Close, (None,)),
            (channel.ChannelFactoryInitialize, (0, "unused-offline")),
        ):
            inspect.signature(function).bind(*args)
        stack.enter_context(patch.object(channel, "ChannelFactoryInitialize", forbidden))
        stack.enter_context(patch.object(channel.ChannelPublisher, "__init__", forbidden))
        stack.enter_context(patch.object(channel.ChannelSubscriber, "__init__", forbidden))
        for module_name, name in (("unitree_sdk2py.g1.loco.g1_loco_client", "LocoClient"),
                                  ("unitree_sdk2py.comm.motion_switcher.motion_switcher_client", "MotionSwitcherClient"),
                                  ("unitree_sdk2py.go2.robot_state.robot_state_client", "RobotStateClient")):
            cls = getattr(importlib.import_module(module_name), name)
            method = {"LocoClient": "GetFsmId", "MotionSwitcherClient": "CheckMode",
                      "RobotStateClient": "ServiceList"}[name]
            query = getattr(cls, method)
            inspect.signature(query).bind(None)
            interfaces[name + "." + method] = str(inspect.signature(query))
            stack.enter_context(patch.object(cls, "__init__", forbidden))
        yield attempted, interfaces


def pack_command(command):
    """独立构造 SDK CRC 的小端、4 字节对齐布局；不是 XCDR 网络载荷。"""
    fields = [command.mode_pr, command.mode_machine]
    for motor in command.motor_cmd:
        fields.extend((motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd, motor.reserve))
    fields.extend(command.reserve)
    fields.append(command.crc)
    return struct.pack("<BBxx" + "Bxxx5fI" * 35 + "5I", *fields)


def reference_crc(command):
    """独立逐位校验 CRC32，排除末尾 CRC 字段，不调用 SDK CRC 内部方法。"""
    raw = pack_command(command)
    crc = 0xFFFFFFFF
    for (word,) in struct.iter_unpack("<I", raw[:-4]):
        for bit in range(31, -1, -1):
            high = crc & 0x80000000
            crc = (crc << 1) & 0xFFFFFFFF
            if high:
                crc ^= 0x04C11DB7
            if word & (1 << bit):
                crc ^= 0x04C11DB7
    return crc


def command_summary(command):
    payload = command.serialize()
    restored = type(command).deserialize(payload)
    if restored.serialize() != payload:
        raise RuntimeError("LowCmd XCDR 往返不一致")
    if restored.crc != reference_crc(command) or reference_crc(restored) != restored.crc:
        raise RuntimeError("SDK CRC 与独立布局校验不一致")
    return {"type_name": command.__idl_typename__, "mode_pr": command.mode_pr,
            "mode_machine": command.mode_machine, "motor_count": len(command.motor_cmd),
            "crc": command.crc, "crc_layout_bytes": len(pack_command(command)),
            "xcdr_bytes": len(payload), "xcdr_sha256": hashlib.sha256(payload).hexdigest(),
            "xcdr_hex": payload.hex(),
            "weight": command.motor_cmd[WEIGHT_SLOT].q,
            "motors": [{"slot": slot, "mode": m.mode, "q": m.q, "dq": m.dq, "tau": m.tau,
                        "kp": m.kp, "kd": m.kd, "reserve": m.reserve}
                       for slot, m in enumerate(restored.motor_cmd)]}


def fake_services():
    return (SimpleNamespace(GetFsmId=lambda: (0, 501)),
            SimpleNamespace(CheckMode=lambda: (0, {"name": "ai"})),
            SimpleNamespace(ServiceList=lambda: (0, [SimpleNamespace(name="g1_arm_example", status=0, protect=False)])))


class RealMessageTransport(SimulatedSDK):
    """内存传输真实 LowState/LowCmd；没有 ChannelFactory 或 DDS 实体。"""
    def __init__(self, clock, baseline, sdk):
        super().__init__(clock, baseline)
        self.binding = sdk
        self.crc = sdk.CRC()
        self.channel_types = []
        self.roundtrip_states = 0
        self.roundtrip_commands = 0

    def ChannelSubscriber(self, topic, message_type):
        required = self.binding.LowStateType if topic == "rt/lowstate" else self.binding.LowCmdType
        if message_type is not required:
            raise RuntimeError("订阅通道传入的不是 SDK 消息类型")
        self.channel_types.append({"topic": topic, "kind": "subscriber", "type_name": message_type.__idl_typename__})
        return super().ChannelSubscriber(topic, message_type)

    def ChannelPublisher(self, topic, message_type):
        if topic != "rt/arm_sdk" or message_type is not self.binding.LowCmdType:
            raise RuntimeError("发布通道的主题或 SDK 消息类型不匹配")
        self.channel_types.append({"topic": topic, "kind": "publisher", "type_name": message_type.__idl_typename__})
        publisher = super().ChannelPublisher(topic, message_type)
        original = publisher.Write
        def write(command):
            if type(command) is not self.binding.LowCmdType or command.crc != self.crc.Crc(command):
                raise RuntimeError("发布实例类型或 SDK CRC 无效")
            payload = command.serialize()
            restored = self.binding.LowCmdType.deserialize(payload)
            if restored.serialize() != payload or restored.crc != self.crc.Crc(restored):
                raise RuntimeError("命令序列化往返或 CRC 失败")
            self.roundtrip_commands += 1
            return original(restored)
        publisher.Write = write
        return publisher

    def _publish_state(self):
        if self.drop_states:
            return
        message = self.binding.LowState()
        message.mode_pr, message.mode_machine = self.mode_pr, self.mode_machine
        message.tick = round(self.clock.now * 1000)
        for offset, slot in enumerate(ARM_SLOTS):
            motor = message.motor_state[slot]
            motor.mode, motor.q, motor.dq = self.motor_modes[offset], self.q[offset], self.dq[offset]
        message.crc = self.crc.Crc(message)
        payload = message.serialize()
        restored = self.binding.LowStateType.deserialize(payload)
        if restored.serialize() != payload or restored.crc != self.crc.Crc(restored):
            raise RuntimeError("状态序列化往返或 CRC 失败")
        self.roundtrip_states += 1
        for subscriber in self.subscribers:
            if not subscriber.closed and subscriber.topic == "rt/lowstate":
                subscriber.callback(restored)


def audit_unitree_replay(plan):
    validate_plan(plan)
    with block_network() as (attempts, interfaces):
        sdk = UnitreeReplaySDK(service_clients=fake_services())
        clock = VirtualClock()
        transport = RealMessageTransport(clock, plan["baseline"]["q"], sdk)
        sdk.offline_transport = transport
        report = execute_replay(plan, SIMULATION_PROFILE, lambda: ReplaySdkPort(sdk, clock), clock, Event())
        if report["status"] != "completed":
            raise RuntimeError("真实 SDK 报文离线回放失败：{}；{}".format(report["errors"], report["cleanup_errors"]))
        if attempts or not all(resource.closed for resource in transport.resources):
            raise RuntimeError("出现被阻断的网络调用或资源未关闭")
        # 所有命令经过真实 SDK CRC + XCDR 往返；选首、中、末帧另做独立 CRC 对照。
        selected = [transport.writes[i] for i in (0, len(transport.writes) // 2, len(transport.writes) - 1)]
        packet_samples = [command_summary(command) for command in selected]
        default = sdk.LowCmd()
        for command in transport.writes:
            if any(command.motor_cmd[i] != default.motor_cmd[i] for i in list(range(15)) + list(range(30, 35))):
                raise RuntimeError("回放意外改写非手臂槽位")
        crc_source = Path(inspect.getfile(sdk.CRC))
        idl_source = Path(inspect.getfile(sdk.LowCmdType))
        sources = [idl_source, Path(inspect.getfile(sdk.LowStateType)), crc_source,
                   Path(inspect.getfile(sdk.LowCmd)), Path(inspect.getfile(sdk.channel))]
        if hasattr(transport.crc, "crc_lib"):
            sources.append(Path(transport.crc.crc_lib._name))
        return {"schema": "g1-arm7-unitree-sdk-offline-audit-v1", "status": "passed",
                "qualification": "offline_sdk_interface_and_packets_only", "hardware_enabled": False,
                "dds_initialized": False, "network_calls_blocked": len(attempts),
                "network_guard_enabled": True, "interface_signatures_bound": True,
                "cyclonedds_version": importlib.metadata.version("cyclonedds"),
                "sdk_sources": [{"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                                for p in sources],
                "interfaces": interfaces, "channel_types": transport.channel_types,
                "service_queries": "injected_read_only_replies_not_robot_rpc",
                "roundtrip_commands": transport.roundtrip_commands, "roundtrip_states": transport.roundtrip_states,
                "packet_samples": packet_samples, "executor_metrics": report["metrics"],
                "release_sent": report["release_sent"], "handover_confirmed": False,
                "resources_closed": True}

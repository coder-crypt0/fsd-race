"""BLOCK 7 (Jetson side) — CAN BRIDGE (owner: Controls & Actuation).

Translates /control/cmd + /safety/ebs_trigger into the CAN frames defined
in the interface spec (Section 9), and republishes STM32 feedback as
/vehicle/status and /wheel_speeds.

CAN map (CAN 2.0B @ 500 kbps):
  0x100 Jetson->STM : steering(i16 mrad) torque(i16 0.1Nm) brake(u8 0-200) flags(u8)
  0x101 Jetson->STM : rolling counter(u8) + CRC8 of 0x100 payload
  0x200 STM->Jetson : actual steering(i16 mrad) rpm(i16) as_state(u8) faults(u16)
  0x201 STM->Jetson : wheel speeds fl/fr/rl/rr (4x u16, 0.01 rad/s)
  0x210 STM->Jetson : ebs pressure(u16 0.01bar) motor_temp(i8) inv_temp(i8) lv(u16 0.01V)

CRC8: poly 0x31, init 0xFF — identical table in firmware/stm32_bridge/main.c.
Runs only on the real vehicle (needs python-can + SocketCAN 'can0').
"""

import struct
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from fsd_msgs.msg import VehicleCmd, VehicleStatus, WheelSpeeds, Heartbeat

from .common import HeartbeatEmitter, qos_reliable


def crc8(data: bytes, poly=0x31, init=0xFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge')
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('bitrate', 500000)

        self._hb = HeartbeatEmitter(self, 'can_bridge')
        self._bus = None
        try:
            import can
            self._bus = can.interface.Bus(
                channel=str(self.get_parameter('channel').value),
                interface='socketcan')
        except Exception as e:
            self.get_logger().fatal(f'CAN bus unavailable: {e}')
            self._hb.set_status(Heartbeat.STATUS_ERROR, 'CAN bus unavailable')

        self._cmd = VehicleCmd()
        self._ebs = False
        self._counter = 0

        self.create_subscription(VehicleCmd, '/control/cmd', self._on_cmd,
                                 qos_reliable(1))
        self.create_subscription(Bool, '/safety/ebs_trigger', self._on_ebs,
                                 qos_reliable(10))
        self._status_pub = self.create_publisher(VehicleStatus, '/vehicle/status',
                                                 qos_reliable(10))
        self._wheels_pub = self.create_publisher(WheelSpeeds, '/wheel_speeds',
                                                 qos_reliable(10))
        self.create_timer(1.0 / 50.0, self._send)

        if self._bus is not None:
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

    def _on_cmd(self, msg):
        self._cmd = msg

    def _on_ebs(self, msg):
        if msg.data:
            self._ebs = True   # latched, matching the supervisor

    # ------------------------------------------------------------------ TX
    def _send(self):
        if self._bus is None:
            return
        import can
        c = self._cmd
        steer_mrad = int(max(-32768, min(32767, round(c.steering_angle * 1000.0))))
        torque_dnm = int(max(-32768, min(32767, round(c.torque_request * 10.0))))
        brake = int(max(0, min(200, round(c.brake_cmd * 200.0))))
        flags = 0x01 if (c.emergency_stop or self._ebs) else 0x00
        payload = struct.pack('<hhBB', steer_mrad, torque_dnm, brake, flags)
        try:
            self._bus.send(can.Message(arbitration_id=0x100, data=payload,
                                       is_extended_id=False))
            check = struct.pack('<BB', self._counter, crc8(payload))
            self._bus.send(can.Message(arbitration_id=0x101, data=check,
                                       is_extended_id=False))
            self._counter = (self._counter + 1) & 0xFF
        except Exception as e:
            self._hb.set_status(Heartbeat.STATUS_ERROR, f'CAN TX failed: {e}')

    # ------------------------------------------------------------------ RX
    def _rx_loop(self):
        while rclpy.ok():
            try:
                msg = self._bus.recv(timeout=0.5)
            except Exception:
                continue
            if msg is None:
                continue
            if msg.arbitration_id == 0x200 and len(msg.data) >= 7:
                steer, rpm, state = struct.unpack_from('<hhB', msg.data, 0)
                faults, = struct.unpack_from('<H', msg.data, 5)
                s = VehicleStatus()
                s.header.stamp = self.get_clock().now().to_msg()
                s.actual_steering_angle = steer / 1000.0
                s.motor_rpm = float(rpm)
                s.as_state = state
                s.fault_flags = faults
                self._status_pub.publish(s)
            elif msg.arbitration_id == 0x201 and len(msg.data) >= 8:
                fl, fr, rl, rr = struct.unpack('<HHHH', msg.data[:8])
                w = WheelSpeeds()
                w.header.stamp = self.get_clock().now().to_msg()
                w.fl, w.fr = fl * 0.01, fr * 0.01
                w.rl, w.rr = rl * 0.01, rr * 0.01
                self._wheels_pub.publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

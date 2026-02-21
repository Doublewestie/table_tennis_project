# motor_control.py（修改后的完整代码）

import serial
import struct
import time
import config

class LingKongMotor:
    def __init__(self, port, baudrate=115200, motor_id=1, simulation=False):
        """
        :param port: 串口号（仅在 simulation=False 时使用）
        :param baudrate: 波特率
        :param motor_id: 电机 ID
        :param simulation: True 表示仿真模式，不连接真实硬件
        """
        self.motor_id = motor_id
        self.simulation = simulation
        self.current_angle = 0.0   # 模拟当前角度（度）
        
        if not simulation:
            # 真实模式：打开串口
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.1)
                print(f"[电机] 已打开串口 {port}")
            except Exception as e:
                raise Exception(f"[电机] 无法打开串口 {port}: {e}")
        else:
            print(f"[电机] 仿真模式启动，电机 ID={motor_id}")

    def _checksum(self, data):
        return sum(data) & 0xFF

    def _send_cmd(self, cmd, data_bytes):
        """仿真模式下不实际发送，仅打印指令"""
        if self.simulation:
            cmd_name = {
                0x80: "电机关闭",
                0xA6: "单圈位置控制",
                0xA8: "增量位置控制",
            }.get(cmd, f"未知命令(0x{cmd:02X})")
            print(f"[仿真电机] 发送命令: {cmd_name}, 数据: {data_bytes.hex() if isinstance(data_bytes, bytes) else data_bytes}")
            # 模拟电机状态更新（根据命令类型模拟）
            if cmd == 0xA6 and len(data_bytes) >= 9:  # 单圈位置控制命令2 有9字节数据（含校验）
                # 解析目标角度（第2-3字节，小端）
                angle_val = data_bytes[1] | (data_bytes[2] << 8)
                self.current_angle = angle_val / 100.0
                print(f"[仿真电机] 模拟角度更新为 {self.current_angle}°")
            return b'\x3E\xA6\x01\x07...'  # 模拟回复（可根据需要简化）
        else:
            # 真实模式：发送指令并等待回复
            header = [0x3E, cmd, self.motor_id, len(data_bytes)]
            header_cs = self._checksum(header)
            frame = bytes(header + [header_cs] + data_bytes)
            self.ser.write(frame)
            resp = self.ser.read(13)  # 大多数回复13字节
            return resp

    def set_single_turn_angle(self, angle_deg, direction=0, max_speed_dps=360):
        """
        单圈位置控制（命令0xA6）
        angle_deg: 0~359.99
        direction: 0顺时针, 1逆时针
        max_speed_dps: 最大速度 (°/s)
        """
        angle_val = int(angle_deg * 100)          # 转为0.01°/LSB
        speed_val = int(max_speed_dps * 100)      # 转为0.01dps/LSB
        data = [
            direction,
            angle_val & 0xFF,
            (angle_val >> 8) & 0xFF,
            0x00,
            speed_val & 0xFF,
            (speed_val >> 8) & 0xFF,
            (speed_val >> 16) & 0xFF,
            (speed_val >> 24) & 0xFF
        ]
        data_cs = self._checksum(data)
        data.append(data_cs)
        return self._send_cmd(0xA6, data)

    def enable(self):
        """使能电机（仿真模式下打印）"""
        if self.simulation:
            print("[仿真电机] 使能")
        else:
            # 真实使能：可发送一个保持当前位置的命令
            self.set_single_turn_angle(0, direction=0, max_speed_dps=360)

    def disable(self):
        """电机关闭命令 0x80"""
        if self.simulation:
            print("[仿真电机] 关闭")
        else:
            self._send_cmd(0x80, [])

    def close(self):
        if not self.simulation and hasattr(self, 'ser'):
            self.ser.close()
            print("[电机] 串口已关闭")
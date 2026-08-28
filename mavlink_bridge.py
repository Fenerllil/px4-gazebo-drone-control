import os
os.environ["MAVLINK20"] = "1"

import time
import math 
import serial
import struct
from pymavlink import mavutil

#порт, который выдал socat
PORT_RX = "/tmp/tty_mavlink"

def quaternion_to_euler_raw(w, x, y, z):
    l0 = +2.0*(w*x + y*z)
    l1 = +1.0 - 2.0*(x**2 + y**2)
    roll = math.atan2(l0, l1)

    l2 = +2.0*(w*y - z*x)
    l2 = max(-1.0, min(1.0, l2))
    pitch = math.asin(l2)

    l3 = +2.0*(w*z + x*y)
    l4 = +1.0 - 2.0*(y**2 + z**2) 
    yaw = math.atan2(l3, l4)
    return roll, pitch, yaw

class MavlinkBridge:
    def __init__(self):
        self.start_time = time.time()
        self.last_1hz_timer = 0.0
        self.mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
        print("Qgs подключен")
        #Чтения бинарного протокола
        try:
            self.ser = serial.Serial(PORT_RX, baudrate=115200, timeout=0.01)
            print(f"MavlinkBridge слушает порт {PORT_RX}")
        except Exception as e:
            print(f"Ошибка открытия порта {PORT_RX}: {e}"); exit(1)

        
        self.last_time = None
        self.last_roll = 0.0 
        self.last_pitch = 0.0 
        self.last_yaw = 0.0 
        self.last_x = 0.0
        self.last_y = 0.0 
        self.last_z = 0.0 
        self.last_Vx = 0.0 
        self.last_Vy = 0.0
        self.last_Vz = 0.0 

    def process_binary_data(self, x, y, z, qw, qx, qy, qz):
        roll, pitch, yaw = quaternion_to_euler_raw(qw, qx, qy, qz)
        boot_ms = int((time.time() - self.start_time) * 1000)
        heading_deg = int(math.degrees(yaw) % 360.0)
        
        self.current_time = time.time()
        if self.last_time == None:
            self.last_roll = roll
            self.last_pitch = pitch 
            self.last_yaw = yaw 
            self.last_x = x
            self.last_y = y 
            self.last_z = z
            self.last_time = time.time()
            return
            
        dt = self.current_time - self.last_time
        if dt <= 0: return
        
        self.current_Vroll = (roll - self.last_roll)/dt
        self.current_Vpitch = (pitch - self.last_pitch)/dt
        self.current_Vyaw = (yaw - self.last_yaw)/dt
        
        self.current_Vx = (x - self.last_x)/dt 
        self.current_Vy = (y - self.last_y)/dt 
        self.current_Vz = (z - self.last_z)/dt 
        
        # формулы ускорений
        self.current_ax = (self.current_Vx - self.last_Vx)/dt
        self.current_ay = (self.current_Vy - self.last_Vy)/dt
        self.current_az = (self.current_Vz - self.last_Vz)/dt
        
        # 1. Отправка в MAVLink
        self.mav.mav.attitude_send(
            boot_ms, roll, pitch, yaw, 
            self.current_Vroll, self.current_Vpitch, self.current_Vyaw
        )
        self.mav.mav.highres_imu_send(
            boot_ms * 1000,
            self.current_ax, self.current_ay, self.current_az,
            self.current_Vroll, self.current_Vpitch, self.current_Vyaw, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 63 
        )
        self.mav.mav.vfr_hud_send(
            airspeed=0.0,
            groundspeed=math.sqrt(self.current_Vx**2 + self.current_Vy**2),
            heading=heading_deg, throttle=50, alt=z, climb=self.current_Vz
        )
        alt_mm = int(z * 1000)
        self.mav.mav.global_position_int_send(
            boot_ms, 0, 0, 0, alt_mm, 
            int(self.current_Vx * 100), int(self.current_Vy * 100), int(self.current_Vz * 100), heading_deg * 100
        )
        self.mav.mav.local_position_ned_send(
            boot_ms, x, y, -z, self.current_Vx, self.current_Vy, -self.current_Vz 
        )
        
        #перезапись
        self.last_x, self.last_y, self.last_z = x, y, z
        self.last_roll = roll
        self.last_pitch = pitch 
        self.last_yaw = yaw 
        self.last_Vx = self.current_Vx
        self.last_Vy = self.current_Vy
        self.last_Vz = self.current_Vz
        self.last_time = self.current_time

    def run(self):
        print("Подключение к 127.0.0.1:14550")
        try: 
            while True:
                now = time.time()
                
                if now - self.last_1hz_timer >= 1.0:
                    self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_AUTOPILOT_PX4, 
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        0, mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                    self.mav.mav.sys_status_send(
                        0, 0, 0, 500, 12000, -1, 100, 0, 0, 0, 0, 0, 0
                    )
                    self.last_1hz_timer = now
                
                # ЧИТАЕМ БИНАРНЫЙ ПРОТОКОЛ ИЗ СOM-ПОРТА
                if self.ser.in_waiting >= 33:
                    byte = self.ser.read(1)
                    if byte == b'\xAA':
                        header = self.ser.read(2)
                        if len(header) == 2:
                            
                            msg_id = header[0]
                            payload_len = header[1]
                            
                            raw_packet = self.ser.read(payload_len + 1)
                            if len(raw_packet) == (payload_len + 1):
                                
                                payload = raw_packet[:payload_len]
                                received_crc = raw_packet[-1]
                                
                                # Проверка контрольной суммы
                                calculated_crc = msg_id ^ payload_len
                                for b in payload: calculated_crc ^= b
                                    
                                if calculated_crc == received_crc and msg_id == 0x01:
                                    
                                    # РАСПАКОВЫВАЕМ И СРАЗУ ПУСКАЕМ В ТВОЙ МАТЕМАТИЧЕСКИЙ БЛОК
                                    x, y, z, qw, qx, qy, qz = struct.unpack('<7f', payload)
                                    self.process_binary_data(x, y, z, qw, qx, qy, qz)
                                    
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nОстановка")

if __name__ == "__main__":
    bridge = MavlinkBridge()
    bridge.run()

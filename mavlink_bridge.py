import os
os.environ["MAVLINK20"] = "1"

import time
import math 
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V 
from pymavlink import mavutil





def quaternion_to_euler(quat):
    # Крен
    w, x, y, z = quat.w, quat.x, quat.y, quat.z
    l0 = +2.0*(w*x + y*z)
    l1 = +1.0 - 2.0*(x**2 + y**2)
    roll = math.atan2(l0, l1)

    # Тангаж 
    l2 = +2.0*(w*y - z*x)
    l2 = max(-1.0, min(1.0, l2))
    pitch = math.asin(l2)

    # Рысканье
    l3 = +2.0*(w*z + x*y)
    l4 = +1.0 - 2.0*(y**2 + z**2) 
    yaw = math.atan2(l3, l4)

    return roll, pitch, yaw

class MavlinkBridge:
    def __init__(self, drone_name="my_object"):
        self.drone_name = drone_name
        self.node = Node()
        self.start_time = time.time()
        self.last_1hz_timer = 0.0
        self.mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
        print("Доехали")
        self.node.subscribe(Pose_V, "/world/default/dynamic_pose/info", self._pose_callback)

    def _pose_callback(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == self.drone_name:
                roll, pitch, yaw = quaternion_to_euler(p.orientation)
                boot_ms = int((time.time() - self.start_time) * 1000)
                # Перевод рысканья из -pi до pi в 0 до 360
                heading_deg = int(math.degrees(yaw) % 360.0)
                
                # 1. Углы 
                self.mav.mav.attitude_send(boot_ms, roll,pitch, yaw, 0.0, 0.0, 0.0)
                """
                self.mav.mav.attitude_quaternion_send(
                    boot_ms,
                    p.orientation.w,
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    0.0, 0.0, 0.0
                )
                """
                # 2.высота и компас на HUD
                self.mav.mav.vfr_hud_send(
                    airspeed=0.0,
                    groundspeed=0.0,
                    heading=heading_deg,
                    throttle=50,
                    alt=p.position.z, 
                    climb=0.0
                )
                #3 Высоту QGc хавает от сюда. Вместо гпс шлем нули, но суем высоту.
                alt_mm = int(p.position.z * 1000)
                self.mav.mav.global_position_int_send(
                    boot_ms,
                    0, 0, 0,  # lat, lon, alt 
                    alt_mm,   # relative_alt идет в боковую панель
                    0, 0, 0, heading_deg * 100
                )
                break 

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
                    # Фиктивный статус систем (чтобы QGC видел 100% батарею и не выдавал ошибок)
                    self.mav.mav.sys_status_send(
                        0, 0, 0, 500, 12000, -1, 100, 0, 0, 0, 0, 0, 0
                    )
                    self.last_1hz_timer = now
                
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nОстановка")

if __name__ == "__main__":
    bridge = MavlinkBridge(drone_name="my_object")
    bridge.run()


                

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
    def __init__(self,drone_name = "my_object"):
        self.drone_name = drone_name
        self.node = Node()

        self.start_time = time.time()
        self.last_heartbeat = 0.0

        self.mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1,source_component=1)
        current_z_topic = "/world/default/dynamic_pose/info"
        self.node.subscribe(Pose_V, current_z_topic, self._pose_callback)

    def get_boot_time_ms(self):
        """Возвращает время работы скрипта в миллисекундах для MAVLink."""
        return int((time.time() - self.start_time) * 1000)

    def _pose_callback(self,msg:Pose_V):
        for p in msg.pose:
            if p.name == self.drone_name:
                roll, pitch, yaw = quaternion_to_euler(p.orientation)
                boot_ms = self.get_boot_time_ms()
                #OTPRAVKA HEARTBEAT
                if time.time() - self.last_heartbeat >= 1.0:
                    self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        0, mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                    self.last_heartbeat = time.time()
                #otpravka yglov
                self.mav.mav.attitude_send(
                    boot_ms, roll, pitch, yaw, 0.0, 0.0, 0.0
                )
                
                #z_mav = -z_gazebo
                self.mav.mav.local_position_ned_send(
                    boot_ms, 
                    p.position.x, p.position.y, -p.position.z, 
                    0.0, 0.0, 0.0
                )
                break
    def run(self):
        try: 
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Остановка")
if __name__ == "__main__":
    bridge = MavlinkBridge(drone_name = "my_object")
    bridge.run()


                

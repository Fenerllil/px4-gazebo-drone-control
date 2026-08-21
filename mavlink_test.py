import os
os.environ["MAVLINK20"] = "1"
import time
import math
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
start_time = time.time()
print("Газ")
while True:
    t = time.time() - start_time
    boot_ms = int(t * 1000)
    
    roll = math.radians(30 * math.sin(t * 2))
    pitch = math.radians(20 * math.cos(t * 2))
    
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_PX4,
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        0, mavutil.mavlink.MAV_STATE_ACTIVE
    )
    mav.mav.sys_status_send(0, 0, 0, 500, 12000, -1, 100, 0, 0, 0, 0, 0, 0)

    mav.mav.attitude_send(boot_ms, roll, pitch, 0.0, 0.0, 0.0, 0.0)

    mav.mav.vfr_hud_send(0.0, 0.0, 0, 50, 2.5, 0.0)
    
    mav.mav.global_position_int_send(boot_ms, 0, 0, 0, 2500, 0, 0, 0, 0)
    
    time.sleep(0.05)
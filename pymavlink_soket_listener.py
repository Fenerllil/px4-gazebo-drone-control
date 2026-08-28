import os
os.environ["MAVLINK20"] = "1"
import math
from pymavlink import mavutil 

mav = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
print("Connected")

try:
    while True:
        msg = mav.recv_match(blocking =True)
        """
        msg = mav.recv_match(type = 'ATTITUDE',blocking = True)
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            print(f"Крен {roll:.2f}, Тангаж {pitch:.2f}")
        """
        print(f"[{msg.get_type()}] -> {msg.to_dict()}")
except KeyboardInterrupt:
    print("Connection stopped")

import time
import serial
import struct
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V 
from protocol import CustomProtocol
PORT_TX = "/tmp/tty_packer" 

### socat -d -d PTY,link=/tmp/tty_packer,raw,echo=0 PTY,link=/tmp/tty_mavlink,raw,echo=0


try:
    ser = serial.Serial(PORT_TX, baudrate=115200)
    print(f"{PORT_TX}")
except Exception as e:
    print(f"Ошибка открытия порта: {e}"); exit(1)

def _pose_callback(msg: Pose_V):
    for p in msg.pose:
        if p.name == "my_object":
            packet = CustomProtocol.pack_pose(p.position.x,p.position.y,p.position.z,
                                              p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z)
            ser.write(packet)

node = Node()
node.subscribe(Pose_V, "/world/default/dynamic_pose/info", _pose_callback)
try:
    while True: time.sleep(1)
except KeyboardInterrupt: print("Выход")

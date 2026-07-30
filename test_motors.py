from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V 

import time 


class TestMotors:
    def __init__(self, drone_name="x500"):
        self.drone_name = drone_name
        self.node = Node()
        self.flag = False
        self.current_z = 0.0
        self.current_vz = 0.0  
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        current_z_topic = "/world/default/dynamic_pose/info"
    
        self.node.subscribe(Pose_V, current_z_topic, self._pose_callback)

        motor_topic = f"/{self.drone_name}/command/motor_speed"
        self.pub = self.node.advertise(motor_topic, Actuators)
        time.sleep(0.5)

    
    def _pose_callback(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == self.drone_name:
                self.current_z = p.position.z
                self.flag = True
                break

    def send_motor_command(self, speeds: list): # -> [800,800,800,800]
        cmd = Actuators()
        cmd.velocity.extend(speeds)
        self.pub.publish(cmd)  
        
    def run(self):
        while not self.flag:
            time.sleep(0.1)

        start_time = time.time()  # 13 cекунд 

        try:
            while time.time() - start_time < 10.0:  # 20 - 13 = 7 
                self.send_motor_command([800.0] * 4)
                print(f"Текущая высота по Z: {self.current_z:.2f} м", end="\r")
                time.sleep(0.01)

        finally:
            self.send_motor_command([0.0] * 4)


if __name__ == "__main__":
    drone = TestMotors()
    drone.run()

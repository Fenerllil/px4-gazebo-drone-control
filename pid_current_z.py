import time 
import math 
import sys 
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V 
from gz.msgs10 import odometry_pb2 

class Pid_Controller:
    def __init__(self,kp,ki,kd):
        self.kp = kp
        self.kd = kd
        self.ki = ki 
        self.all_errors = 0.0
        self.integral = 0.0 
        self.last_error =  0.0
        self.integral = 0.0
        self.last_time = time.time()

    
    def calc(self,error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0001:
            dt = 0.001
        self.integral += (error * dt) 
        self.integral = max(-100.0, min(100.0, self.integral))

        derivative = (error - self.last_error) / dt

        self.last_error = error
        self.start_time = current_time 

        return ((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))
        

class Drone:
    def __init__(self, drone_name = "my_object"):
        self.drone_name = drone_name 
        self.node = Node()
        self.target_z = 2.0
        self.current_z = 0.0 
        self.flag = False

        self.start_rmp = 600.0

        self.pid_res = Pid_Controller(kp=80.0,ki = 0.1,kd=5.0)
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

    def send_motor_command(self, speeds: list): 
        cmd = Actuators()
        cmd.velocity.extend(speeds)
        self.pub.publish(cmd) 

    def run(self):
    
        while not self.flag:
            time.sleep(0.1)
        start_time = time.time()
        print("Dron find")
        try:
            while time.time() - start_time <= 20.0:
                error = self.target_z - self.current_z
                correction = self.pid_res.calc(error)
                self.current_rmp = self.start_rmp + correction 
                self.current_rmp = max(0.0,min(1000.0,self.current_rmp))
                self.send_motor_command([self.current_rmp]*4)
                time.sleep(0.01)

        finally: 
            self.send_motor_command([0.0]*4)

if __name__ == "__main__":
    drone = Drone(drone_name="my_object")
    drone.run()
import time 
import math 
import sys 
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V 

class Pid_Controller:
    def __init__(self,kp,ki,kd,limit = 50.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki 
        self.integral = 0.0 
        self.last_error =  0.0
        self.limit = limit
        #self.last_time = time.time()

    def reset(self):
        self.last_error = 0.0 
        self.integral = 0.0 

    def calc(self,error):
        #current_time = time.time()
        #dt = current_time - self.last_time
        dt = 0.01
        if dt <= 0.0001:
            dt = 0.001
        self.integral += (error * dt) 
        self.integral = max(-self.limit,min(self.limit, self.integral))

        derivative = (error - self.last_error) / dt

        self.last_error = error
        #self.last_time = current_time 

        return ((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))
        
def quaternion_to_euler(quat):
    #Крен
    w,x,y,z = quat.w,quat.x,quat.y,quat.z
    l0 = +2.0*(w*x + y*z)
    l1 = +1.0 - 2.0*(x**2 + y**2)
    roll = math.atan2(l0,l1)

    #Тангаж 
    l2 = +2.0*(w*y - z*x)
    l2 = max(-1.0,min(1,0,l2))
    pitch = math.asin(l2)


    #Рысканье
    l3 = +2.0*(w*z + x*y)
    l4 = +1.0 - 2.0*(y**2 - z**2) 
    yaw = math.atan2(l3,l4)

    return roll, pitch, yaw


class Drone:
    def __init__(self, drone_name = "my_object"):
        self.drone_name = drone_name 
        self.node = Node()


        self.target_z = 5.0
        self.current_z = 0.0

        self.target_roll = 0.0
        self.current_roll = 0.0 

        self.target_pitch = 0.0 
        self.current_pitch = 0.0 

        self.target_yaw = 0.0 
        self.current_yaw = 0.0

        self.flag = False
        self.min_rmp = 0.0
        self.max_rmp = 1000.0
        self.start_rmp = 600.0

        self.pid_Tdes = Pid_Controller(kp=50.0,ki = 0.5,kd = 15.0)

        self.pid_roll = Pid_Controller(kp =20.0, ki =0.0, kd = 3.0)
        self.pid_pitch = Pid_Controller(kp= 20.0,ki = 0.0,kd = 3.0)
        self.pid_yaw = Pid_Controller(kp = 5.0,ki = 0.0,kd = 1.0)

        current_z_topic = "/world/default/dynamic_pose/info"
        self.node.subscribe(Pose_V, current_z_topic, self._pose_callback)


        motor_topic = f"/{self.drone_name}/command/motor_speed"
        self.pub = self.node.advertise(motor_topic, Actuators)
        time.sleep(0.5) 



    def _pose_callback(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == self.drone_name:
                self.current_z = p.position.z
                self.current_roll,self.current_pitch,self.current_yaw = quaternion_to_euler(p.orientation)
                self.flag = True
                break

    def send_motor_command(self, speeds: list): 
        #Надо задать ограничение на моторах
        clamped_speeds = [max(self.min_rmp, min(self.max_rmp, float(s))) for s in speeds]
        cmd = Actuators()
        cmd.velocity.extend(clamped_speeds)
        self.pub.publish(cmd) 

    def run(self):
    
        while not self.flag:
            time.sleep(0.1)

        self.pid_Tdes.reset()
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()

        self.target_yaw = self.current_yaw

        start_time = time.time()
        print("Dron find")
        #self.max_height = 0.0 
        try:
            while time.time() - start_time <= 20.0:
                error = self.target_z - self.current_z
                correction = self.pid_Tdes.calc(error)
                current_rmp = self.start_rmp + correction 

                error_roll = self.target_roll - self.current_roll
                rmp_roll = self.pid_roll.calc(error_roll)
                error_pitch = self.target_pitch - self.current_pitch
                rmp_pitch = self.pid_pitch.calc(error_pitch)
                error_yaw = self.target_yaw - self.current_yaw
                rmp_yaw = self.pid_yaw.calc(error_yaw)



                #print(f"Значение коррекции{self.current_rmp:.2f}")
                #self.max_height = max(self.max_height,self.current_z)
                
                m0 = current_rmp - rmp_roll - rmp_pitch - rmp_yaw
                m1 = current_rmp + rmp_roll + rmp_pitch - rmp_yaw
                m2 = current_rmp + rmp_roll - rmp_pitch + rmp_yaw
                m3 = current_rmp - rmp_roll + rmp_pitch + rmp_yaw
                self.send_motor_command([m0,m1,m2,m3])
                
                
                time.sleep(0.01)

        finally: 
            #print(self.max_height)
            self.send_motor_command([0.0]*4)

if __name__ == "__main__":
    drone = Drone(drone_name="my_object")
    drone.run()
import time 
import math 
import sys 
import matplotlib.pyplot as plt 
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V 

class Pid_Controller:
    def __init__(self, kp, ki, kd, limit=50.0, alpha=0.2):
        self.kp = kp
        self.kd = kd
        self.ki = ki 
        self.integral = 0.0 
        self.last_error =  0.0
        self.limit = limit
        self.alpha = alpha 
        self.filtered_derivative = 0.0
        
        # Переменные для графиков
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def reset(self):
        self.last_error = 0.0 
        self.integral = 0.0 
        self.filtered_derivative = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def calc(self, error):
        dt = 0.01
        if dt <= 0.0001:
            dt = 0.001
            
        self.integral += (error * dt) 
        self.integral = max(-self.limit, min(self.limit, self.integral))

        derivative = (error - self.last_error) / dt

        self.last_error = error
        self.filtered_derivative = (self.alpha * derivative) + ((1.0 - self.alpha) * self.filtered_derivative)
        
        # Переменные для графиков 
        self.last_p = self.kp * error
        self.last_i = self.ki * self.integral
        self.last_d = self.kd * self.filtered_derivative
        
        return self.last_p + self.last_i + self.last_d
        
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


class Drone:
    def __init__(self, drone_name="my_object"):
        self.drone_name = drone_name 
        self.node = Node()

        self.target_z = 7.0
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
        self.start_rmp = 720.0

        self.pid_Tdes = Pid_Controller(kp=30.0, ki=1.5, kd=45.0, limit=100.0, alpha=0.15)
        self.pid_roll = Pid_Controller(kp=16.0, ki=0.0, kd=3.5, alpha=0.3)
        self.pid_pitch = Pid_Controller(kp=16.0, ki=0.0, kd=3.5, alpha=0.3)
        self.pid_yaw = Pid_Controller(kp=4.0, ki=0.0, kd=0.5)

        current_z_topic = "/world/default/dynamic_pose/info"
        self.node.subscribe(Pose_V, current_z_topic, self._pose_callback)

        motor_topic = f"/{self.drone_name}/command/motor_speed"
        self.pub = self.node.advertise(motor_topic, Actuators)
        time.sleep(0.5) 

    def _pose_callback(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == self.drone_name:
                self.current_z = p.position.z
                self.current_roll, self.current_pitch, self.current_yaw = quaternion_to_euler(p.orientation)
                self.flag = True
                break

    def send_motor_command(self, speeds: list): 
        clamped_speeds = [max(self.min_rmp, min(self.max_rmp, float(s))) for s in speeds]
        cmd = Actuators()
        cmd.velocity.extend(clamped_speeds)
        self.pub.publish(cmd) 

    def plot_data(self, history):
        #Функция для отрисовки собранных данных
        t = history['time']
        
        plt.figure(figsize=(10, 12))
        
        # 1. График компонент ПИД-регулятора
        plt.subplot(3, 1, 1)
        plt.plot(t, history['p'], label='P')
        plt.plot(t, history['i'], label='I')
        plt.plot(t, history['d'], label='D')
        plt.title('Компоненты ПИД-регулятора высоты от времени')
        plt.ylabel('Управляющее воздействие')
        plt.legend()
        plt.grid(True)

        # 2. График высоты от времени
        plt.subplot(3, 1, 2)
        plt.plot(t, history['z'], label='Текущая высота', color='blue')
        plt.axhline(y=self.target_z, color='r', linestyle='--', label='Целевая высота')
        plt.title('Высота от времени')
        plt.ylabel('Высота (м)')
        plt.legend()
        plt.grid(True)

        # 3. График оборотов от времени
        plt.subplot(3, 1, 3)
        plt.plot(t, history['rpm'], label= 'обороты (current_rmp)', color='green')
        plt.title('Обороты от времени')
        plt.xlabel('Время (с)')
        plt.ylabel('RPM')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()

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
        self.max_height = 0.0 
        
        # Словарь для значений
        history = {
            'time': [],
            'z': [],
            'rpm': [],
            'p': [],
            'i': [],
            'd': []
        }
        
        try:
            while time.time() - start_time <= 60.0:
                current_time_sim = time.time() - start_time
                
                error = self.target_z - self.current_z
                correction = self.pid_Tdes.calc(error)
                
                current_rmp = self.start_rmp + correction 
                current_rmp = max(400.0, min(800.0, current_rmp))
                
                error_roll = self.target_roll - self.current_roll
                rmp_roll = self.pid_roll.calc(error_roll)
                error_pitch = self.target_pitch - self.current_pitch
                rmp_pitch = self.pid_pitch.calc(error_pitch)
                error_yaw = self.target_yaw - self.current_yaw
                rmp_yaw = self.pid_yaw.calc(error_yaw)

                self.max_height = max(self.max_height, self.current_z)
                
                
                history['time'].append(current_time_sim)
                history['z'].append(self.current_z)
                history['rpm'].append(current_rmp)
                history['p'].append(self.pid_Tdes.last_p)
                history['i'].append(self.pid_Tdes.last_i)
                history['d'].append(self.pid_Tdes.last_d)
                
                m0 = current_rmp - rmp_roll - rmp_pitch - rmp_yaw
                m1 = current_rmp + rmp_roll + rmp_pitch - rmp_yaw
                m2 = current_rmp + rmp_roll - rmp_pitch + rmp_yaw
                m3 = current_rmp - rmp_roll + rmp_pitch + rmp_yaw
                self.send_motor_command([m0, m1, m2, m3])
                
                # Закомментировано, чтобы не спамить в консоль во время работы
                # print(f"Высота {self.current_z: .2f}")
                
                time.sleep(0.01)

        finally: 
            print(f"Максимальная высота: {self.max_height}")
            self.send_motor_command([0.0]*4)
            # Строим графики 
            self.plot_data(history)

if __name__ == "__main__":
    drone = Drone(drone_name="my_object")
    drone.run()
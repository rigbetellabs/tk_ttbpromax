import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import time
from math import pi

motor_rpm = 75             #   max rpm of motor on full voltage 
wheel_diameter = 0.081        #   in meters
wheel_separation = 0.25     #   in meters
max_pwm_val = 255          #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 0            #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec
max_angle = (max_speed*2)/wheel_separation          #   rad/sec

lPWM=Int32()
rPWM=Int32()
lDIR=Bool()
rDIR=Bool()

def stop(self):
    global lPWM, rPWM, lDIR, rDIR
    lPWM.data=0
    rPWM.data=0
    lDIR.data=True
    rDIR.data=True
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)
    self.ldir_pub.publish(lDIR)
    self.rdir_pub.publish(rDIR)
    
def wheel_vel_executer(self, left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    global lPWM, rPWM, lDIR, rDIR

    lspeedPWM = max(min(((abs(left_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((abs(right_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    lPWM.data=int(lspeedPWM)
    rPWM.data=int(rspeedPWM)

    
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)
    
    if left_speed >= 0 :
        lDIR.data=True
    else :
        lDIR.data=False
        
        
    if right_speed >= 0 :
        rDIR.data=True
    else :
        rDIR.data=False

    self.rdir_pub.publish(rDIR)
    self.ldir_pub.publish(lDIR)

class Differential(Node):
    def __init__(self):
        super().__init__('differential')

        self.vel_subscription= self.create_subscription(Twist,"cmd_vel",self.callback,10)
        self.vel_subscription
        self.lpwm_pub = self.create_publisher(Int32, 'lpwm', 10)
        self.rpwm_pub = self.create_publisher(Int32, 'rpwm', 10)
        self.ldir_pub = self.create_publisher(Bool, 'ldir', 10)
        self.rdir_pub = self.create_publisher(Bool, 'rdir', 10)
    
    def callback(self, data):
        
        global wheel_radius
        global wheel_separation
        
        linear_vel = data.linear.x                  # Linear Velocity of Robot
        angular_vel = data.angular.z                # Angular Velocity of Robot


        VrplusVl  = 2 * linear_vel
        VrminusVl = angular_vel * wheel_separation
        
        right_vel = ( VrplusVl + VrminusVl ) / 2      # right wheel velocity along the ground
        left_vel  = VrplusVl - right_vel              # left wheel velocity along the ground
        
        # print (str(left_vel)+"\t"+str(right_vel))
        
        if (left_vel == 0.0 and right_vel == 0.0):
            stop(self)
        else:
            wheel_vel_executer(self, left_vel, right_vel)

def main(args=None):
    
  rclpy.init(args=args)
  differential_drive = Differential()
  rclpy.spin(differential_drive)
  differential_drive.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
    print('Tortoisebot Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    print('Max Angular Speed:\t'+str(max_angle)+' rad/sec')
    main()
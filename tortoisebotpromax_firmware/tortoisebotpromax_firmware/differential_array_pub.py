import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from std_msgs.msg import Int32, Bool
from std_msgs.msg import Int32MultiArray
import time
from math import pi


motor_rpm = 60             #   max rpm of motor on full voltage 
wheel_diameter = 0.081        #   in meters
wheel_separation = 0.242     #   in meters
max_pwm_val = 255          #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 0            #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec

motors_data=Int32MultiArray()
motors_data.data=[0,0,1,1]


def stop(self):
    global motors_data
    motors_data.data[0]=0 #lpwm
    motors_data.data[1]=0 #rpwm
    motors_data.data[2]=1 #ldir
    motors_data.data[3]=1 #rdir
    self.motors_data_pub.publish(motors_data)

    
def wheel_vel_executer(self, left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    global motors_data

    lspeedPWM = max(min(((abs(left_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((abs(right_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    motors_data.data[0]=int(lspeedPWM)
    motors_data.data[1]=int(rspeedPWM)


    
    if left_speed >= 0 :
        motors_data.data[2]=1
    
    else :
        motors_data.data[2]=0
       
        
    if right_speed >= 0 :
        motors_data.data[3]=1
  
    else :
        motors_data.data[3]=0
       

    self.motors_data_pub.publish(motors_data)

class Differential(Node):
    def __init__(self):
        super().__init__('differential')

        self.vel_subscription= self.create_subscription(Twist,"cmd_vel",self.callback,10)
        self.vel_subscription

        self.motors_data_pub = self.create_publisher(Int32MultiArray, 'motors_data', 10)
    
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
    main()

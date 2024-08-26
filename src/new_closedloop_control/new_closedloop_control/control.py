
# code to run the robot from (0,0) to (10,10) in a straight line with yaw control using imu sensor data

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
import math


# declaring global variables to store error and control input values
error=[]
ctrl=[] 
imu_l_acc=[]
imu_l_acc.append(0)
errvel=[]


class SubPub(Node):

    def __init__(self,dx,dy,t):
        
        super().__init__('control_publisher_subscriber')
        
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)     ## publisher to publish position inputs to the steering joint
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)  ## publisher to publish velocity inputs to the wheels
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,              ## qos profile of IMU sensor (Same profile is defined for the imu subscriber)
        history=HistoryPolicy.KEEP_LAST,
        depth=10
        )
        self.imu_subcriber = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)  ##subscriber to subscribe to the IMU sensor data

        self.timer_period = 0.1     
        self.i = 0                      
        self.distance = math.sqrt((dx*dx)+(dy*dy))  
        print(dx,"  ",dy)
        print(self.distance)                      #Timer to run the robot for defined distance in defined time
        self.time = t                  
        self.real_time_factor = 0.83

        self.timer = self.create_timer(self.timer_period, self.timer_callback)



    def timer_callback(self):
        
        msg1 = Float64MultiArray()

        if self.i<=(self.time/self.real_time_factor):
            print(imu_l_acc[0])
            
            if self.i==0:
                err=0                                                     # Calculating error from imu linear acceleration input
            else:
                err=2*float(imu_l_acc[0]*0.01)-(0.09428*0.1)
            
            vel=float((10*(0.094280+(err)))/0.1)
            errvel.append(err)                                            # Calculating the angular velocity inputs to the wheels 
            msg1 = Float64MultiArray(data=[-vel,-vel,-vel,-vel])          # Publishing the angular velocity inputs to the wheels
            self.wheel_velocities_pub.publish(msg1)
            print("Publishing Velocities: ",msg1.data)

        if self.i>(self.time/self.real_time_factor):
            msg1 = Float64MultiArray(data=[0.0,0.0,0.0,0.0])              # Stopping the robot after desired position is reached
            self.wheel_velocities_pub.publish(msg1)
            print("Publishing Velocities: ",msg1.data)
            h=len(error)                                                 
            st=(self.time)/h
            s1=np.arange(0,h*st,st)

            figure, axis = plt.subplots(3) 
            axis[0].plot(s1,error)
            axis[0].set_title("Orientation Error vs Time Graph")
            axis[0].set_xlabel("Time")
            axis[0].set_ylabel("Orientation Error")
                                                                         # Plotting the error vs time and input vs time graphs
            axis[1].plot(s1,ctrl)
            axis[1].set_title("Steering Control Input vs Time Graph")
            axis[1].set_xlabel("Time")
            axis[1].set_ylabel("Orientation/Steering Control Input")

            hvel=len(errvel)                                                 
            stvel=(self.time)/hvel
            s1vel=np.arange(0,hvel*stvel,stvel)

            axis[2].plot(s1vel,errvel)
            axis[2].set_title("Position Error vs Time Graph")
            axis[2].set_xlabel("Time")
            axis[2].set_ylabel("Position Error")

            plt.show()

            exit()
        
        self.i += self.timer_period


    
    def imu_callback(self,msg):
        
        msg2 = Float64MultiArray()

        a=msg.orientation.w
        b=msg.orientation.x
        c=msg.orientation.y                                              # Extracting the quaternion orientation values from the IMU sensor output
        d=msg.orientation.z
        p = 2 * (a * d + b * c)
        q = 1-(2*((c*c)+(d*d)))
        yaw = math.atan2(p,q)

        if yaw!=0:
            kp=5
            yaw_desired=-0.7853
            e=yaw-yaw_desired                                       # Calculating the error by the formula (present yaw - desired yaw) where the desired yaw is 45 degrees as measured from positive y-axis in CW direction(desired yaw = -0.7853 radians. Hence error = present-(-0.7853) = present + 0.7853)
            inp=float(0.785*kp*e) 
            msg2 = Float64MultiArray(data=[inp,inp])
            self.joint_position_pub.publish(msg2)                   # Publishing the position inputs to steering
            print("Publishing Steering Position: ",msg2.data)                     
            error.append(e)
            ctrl.append(inp)
        
        imu_l_acc[0]=msg.linear_acceleration.x
        


def main(args=None):


    rclpy.init(args=args)

    control_subpub = SubPub(10,10,15)                       

    rclpy.spin(control_subpub)

    control_subpub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
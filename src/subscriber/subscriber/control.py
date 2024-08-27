import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Subscriber(Node):

    def __init__(self):
        
        super().__init__('Input_subscriber')
        
        self.position_subscriber_sub = self.create_subscription(Float64MultiArray, '/position_controller/commands', self.position_callback, 10) 
        self.velocity_subscriber = self.create_subscription(Float64MultiArray, '/velocity_controller/commands', self.velocity_callback, 10) 


    def position_callback(self,msg1):
        print("Subscribed steering position data: ",msg1.data)
    
    def velocity_callback(self,msg2):
        print("Subscribed velocity data: ",msg2.data)
        



def main(args=None):


    rclpy.init(args=args)

    input_pub =Subscriber()

    rclpy.spin(input_pub)

    input_pub.destroy_node()

    rclpy.shutdown()
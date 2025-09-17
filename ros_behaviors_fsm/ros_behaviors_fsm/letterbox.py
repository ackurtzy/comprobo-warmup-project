"""This is a ROS node that uses python and ROS to teleop into 
the robot."""

import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistStamped,Vector3
from neato2_interfaces.msg import Bump

class Letterbox(Node):
    """This is a teloperation node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the TeleopNode node. No inputs."""
        super().__init__('letterbox_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.bump_bool = 0
       	self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def steer(self):
        """Takes key input and drives in a direction according to the input
            Legend: 
            w - forward 
            a - left 
            s - backwards
            d - right 
            any other key - stop"""
            
        if self.key == "w": 
            self.drive(0.2, 0.0)
        elif self.key == "a": 
            self.drive(0.0, 0.3)
        elif self.key == "s": 
            self.drive(-0.2, 0.0)
        elif self.key == "d": 
            self.drive(0.0, -0.3)
        elif self.key == '\x03':
            rclpy.shutdown()
        else:
            self.key = None
            self.drive(0.0, 0.0)

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)   
    

    def run_loop(self):
        """Prints a message to the terminal."""
        self.getKey()
        self.steer()



def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Letterbox()    # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()

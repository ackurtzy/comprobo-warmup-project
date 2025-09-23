"""This is a ROS node that uses python and ROS to teleop into 
the robot."""

import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump

class TeleopNode(Node):
    """
    A ROS2 node that allows keyboard teleoperation of a robot
    using the WASD keys. 

    Inherits from:
        rclpy.node.Node

    Publishes to cmd_vel to control neato velocity 
    """
    def __init__(self):
        """
        Initializes the TeleopNode.
        Sets up a publisher for cmd_vel and timer for main 
        thread. Initialized key and saves settings. 
        """
        super().__init__('teleop_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
       	self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def steer(self):
        """
        Processes the most recent key press and sends the appropriate velocity command
        via the drive function. 

        Controls:
            - W: Forward
            - A: Turn left
            - S: Backward
            - D: Turn right
            - Ctrl+C: Shutdown
            - Any other key: Stop
        """
        if self.key == "w": 
            self.drive(0.5, 0.0)
        elif self.key == "a": 
            self.drive(0.0, 0.5)
        elif self.key == "s": 
            self.drive(-0.5, 0.0)
        elif self.key == "d": 
            self.drive(0.0, -0.5)
        elif self.key == '\x03':
            rclpy.shutdown()
        else:
            self.key = None
            self.drive(0.0, 0.0)


    def drive(self, linear, angular):
        """
        Publishes a Twist message with the specified linear and angular velocities.

        Args:
            linear (float): Linear velocity in m/s.
            angular (float): Angular velocity in radians/s.
        """    
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


    def getKey(self):
        """
        Captures a single key press from the terminal.

        !!NOTE!! THIS FUNCTION *IS* BLOCKING. HOWEVER SINCE 
        THE ENTIRE NODE IS JUST FOR TELEOPERATION IT DOESN'T 
        REALLY MATTER HERE. NEEDS TO BE EDITED FOR FSM. SEE 
        THREADED KEY INPUT IN LETTERBOX. 

        Stores the result in self.key. Restores terminal settings after read.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        # also we need to revert to original settings 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) 


    def run_loop(self):
        """
        Reads the latest key press and performs the corresponding movement.
        """
        self.getKey()
        self.steer()



def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = TeleopNode()    # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup


if __name__ == '__main__':
    main()

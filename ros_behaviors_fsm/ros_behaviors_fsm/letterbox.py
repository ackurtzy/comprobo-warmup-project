"""This is a ROS node that uses python and ROS to teleop into 
the robot."""

import rclpy
from rclpy.node import Node
import threading
import math
from time import sleep

import tty
import select
import sys
import termios

import tkinter as tk
from tkinter import simpledialog
from pyhershey import glyph_factory

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,Vector3


class Letterbox(Node):
    """This is a teloperation node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the TeleopNode node. No inputs."""
        super().__init__('letterbox_node')
        print('Press "Enter" to enter a word.')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.position = [0,0,0]
        self.user_input = None
        self.bump_bool = 0
       	self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.process_odom, 10)

        self.key = None
        self._stop_key_thread = False
        self._key_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._key_thread.start()


    def process_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q) 
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        # print(self.position)


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


    def go_to_point(self, desired_x, desired_y):
        current_x, current_y, current_angle = self.position
        
        print("Current x is: ", current_x)
        print("Current y is: ", current_y)
        print("Des x is: ", desired_x)
        print("Des y is: ", desired_y)

        x = desired_x - current_x
        y = desired_y - current_y

        desired_angle = math.atan2(y, x)
        print(current_angle, desired_angle)

        current_angle = current_angle % (2 * math.pi)
        desired_angle = desired_angle % (2 * math.pi)
        
        rotation_needed = (desired_angle - current_angle) % (2*math.pi)

        angular_vel = 0.5
        lin_velocity = 0.3

        if rotation_needed < math.pi: 
            self.drive(linear=0.0, angular=angular_vel)
            sleep((rotation_needed / angular_vel))

        else: 
            rotation_needed = (2* math.pi - rotation_needed)
            self.drive(linear=0.0, angular=-angular_vel)
            sleep((rotation_needed / angular_vel))
        self.drive(linear=0.0, angular=0.0)


        distance = math.sqrt((x)**2 + (y)**2)
        self.drive(linear=lin_velocity, angular=0.0)
        sleep((distance / lin_velocity))
        self.drive(linear=0.0, angular=0.0)
   

    def collect_input(self):
        root = tk.Tk()
        root.withdraw()  

        self.user_input = simpledialog.askstring("Input", "Enter your word")

        if self.user_input is not None:
            strokes = []
            print(f"You entered: {self.user_input}")
            for character in self.user_input:
                strokes.append(glyph_factory.from_ascii(character, 'roman_simplex').segments)
            return strokes
        else:
            print("User cancelled.")
        

    def draw_letter(self):
        strokes_list = self.collect_input()
        self.go_to_point(0,0)
        for character in strokes_list:
            print(character)
            for segment in character: 
                print(segment)
                for point in segment:
                    rclpy.spin_once(self, timeout_sec=1)
                    print(((point[0])/8, (point[1]/8)))
                    self.go_to_point((point[0])/8, (point[1]/8))
            self.go_to_point(0,0)
        self.go_to_point(0,0)


    

    def _keyboard_listener(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not self._stop_key_thread:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self.key = sys.stdin.read(1)
                    # Debug:
                    print(f"Key captured in thread: {repr(self.key)}", flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))


    def run_loop(self):
        """Prints a message to the terminal."""
        if self.key == '\n':
            if self.key == '\n':
                threading.Thread(target=self.draw_letter, daemon=True).start()
                self.key = None
        else:
            self.key = None



def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Letterbox()         # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()


    
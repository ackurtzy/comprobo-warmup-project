"""This is a ROS node that uses python and ROS to draw letters. 
It is activated when the user hits "enter", and then takes user imput."""

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
from geometry_msgs.msg import Twist


class Letterbox(Node):
    """
    A ROS2 node for controlling our Neato to draw letters.

    This node has a thread that listens to keystrokes, and upon hearing an "Enter"
    keypress it brings up a terminal for user input. It then converts each character 
    into stroke data using Hershey fonts and moves the robot accordingly to "draw" 
    the letters. It draws the letters in the same space on top of each other and 
    in reference to the world origin (its starting position).

    Inherits:
        rclpy.node.Node: Base class for ROS2 Python nodes.
    """
    def __init__(self):
        """
        Initializes the Letterbox node.
        Starts keyboard listener and letter drawing threads. 
        Initializes position tracking and control flags. 

        Publishers: `cmd_vel`
        Subscribers: `/odom`
        """

        super().__init__('letterbox_node')
        print('Press "Enter" to enter a word to draw.')

        # Declare variables, subscribers/publishers, and start timer
        timer_period = 0.1
        self.position = [0,0,0]
        self.user_input = None
        
       	self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.process_odom, 10)

        # Set up keystroke thread 
        self.key = None
        self._stop_key_thread = False
        self._key_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._key_thread.start()

        # Set up letter thread 
        self._stop_letter_thread = False
        self.activate_letter = False
        self._letter_thread = threading.Thread(target=self._draw_letter, daemon=True)
        self._letter_thread.start()


    def process_odom(self, msg):
        """
        Callback function for /odom subscription.

        Updates the robot's current position and orientation based on Odometry data.

        Args:
            msg (Odometry): The Odometry message containing pose and orientation.
        """
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q) 
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        # print(self.position)


    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (Float): the linear velocity in m/s
            angular (Float): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


    def go_to_point(self, desired_x, desired_y):
        """
        Moves the robot to a desired 2D point using current odometry information.

        The robot first rotates in place to face the target, then drives straight.

        Args:
            desired_x (float): Target X position.
            desired_y (float): Target Y position.
        """
        current_x, current_y, current_angle = self.position
        
        # print("Current x is: ", current_x)
        # print("Current y is: ", current_y)
        # print("Des x is: ", desired_x)
        # print("Des y is: ", desired_y)

        # First, we calculate the desired orientation to drive in 
        # odom gives us quanternion, not yaw (z direction)
        x = desired_x - current_x
        y = desired_y - current_y

        desired_angle = math.atan2(y, x)
        print(current_angle, desired_angle)

        current_angle = current_angle % (2 * math.pi)
        desired_angle = desired_angle % (2 * math.pi)
        
        rotation_needed = (desired_angle - current_angle) % (2*math.pi)

        angular_vel = 0.5
        lin_velocity = 0.3

        # then we can perform our actual rotation
        if rotation_needed < math.pi: 
            self.drive(linear=0.0, angular=angular_vel)
            sleep((rotation_needed / angular_vel))

        else: 
            rotation_needed = (2* math.pi - rotation_needed)
            self.drive(linear=0.0, angular=-angular_vel)
            sleep((rotation_needed / angular_vel))
        self.drive(linear=0.0, angular=0.0)

        # calculate needed distance and drive forward 
        distance = math.sqrt((x)**2 + (y)**2)
        self.drive(linear=lin_velocity, angular=0.0)
        sleep((distance / lin_velocity))

        # set speed to zero 
        self.drive(linear=0.0, angular=0.0)
   

    def collect_input(self):
        """
        Prompts the user with a pop up terminal to enter a word, then converts each
        character into stroke segments using Hershey fonts.

        Returns:
            list: A list of strokes for each character in the input string.
        """
        root = tk.Tk()
        root.withdraw()  

        self.user_input = simpledialog.askstring("Input", "Enter your word")

        if self.user_input is not None and self.user_input.strip():
            strokes = []
            print(f"Now Drawing: {self.user_input}")
            for character in self.user_input:
                try: 
                    strokes.append(glyph_factory.from_ascii(character, 'roman_simplex').segments)
                except: 
                    print("Invalid Character(s)!")
            return strokes
        else:
            print("User cancelled.")
            return []
        

    def _draw_letter(self):
        """
        Threaded function that activates when self.activate_letter
        is toggled. Runs in a while loop until then. 

        When self.activate_letter is True, it:
        - Prompts user for a word and gets strokes with collect input 
        - Moves the robot along the strokes with go_to_point
        - Resets to the home position and sets self.activate_letter to False
        """
        while not self._stop_letter_thread:
            # print("Letter thread check!")
            if self.activate_letter:
                # print("Letter initatied!")
                strokes_list = self.collect_input()
                self.go_to_point(0,0)
                if strokes_list: 
                    for character in strokes_list:
                        # print(character)
                        for segment in character: 
                            # print(segment)
                            for point in segment:
                                # print(((point[0])/10, (point[1]/10)))
                                self.go_to_point((point[0])/10, (point[1]/10))
                        self.go_to_point(0,0)
                    self.go_to_point(0,0)
                    self.activate_letter = False
    

    def _keyboard_listener(self):
        """
        Threaded function to listen for keyboard input by user.

        Sets `self.key` when a key is pressed. When ternimated, it 
        resets the terminal to echo and be normal. 
        """
        # print("Letter thread check!")

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd) 
        try: 
            tty.setcbreak(sys.stdin.fileno())  
            while not self._stop_key_thread:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self.key = sys.stdin.read(1)
                    # print(f"Key captured in thread: {repr(self.key)}", flush=True)
        finally:
            old[3] = old[3] | termios.ECHO
            termios.tcsetattr(fd, termios.TCSADRAIN, old)     
            


    def run_loop(self):
        """
        Checks for user to hit the enter key to activate letter drawing.
        Resets the key state otherwise. 
        """
        if self.key == '\n':
            self.activate_letter = True
            # print("Thread Activated!")
            self.key = None
        else:
            self.key = None



def quaternion_to_yaw(q):
    """
    Converts a quaternion into a yaw (z rotation).

    Args:
        q (geometry_msgs.msg.Quaternion): Orientation as a quaternion.

    Returns:
        float: yaw in radians.
    """
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.

    When control-C is hit, it stops the threads (therby resetting the terminal)
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Letterbox()         # Create our Node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        node._stop_key_thread = True
        node._stop_letter_thread = True

        node._key_thread.join(timeout=1.0)
        node._letter_thread.join(timeout=1.0)

        rclpy.shutdown()




if __name__ == '__main__':
    main()


    
"""
Keyboard teleop node with finite-state behaviors (shape, letterbox, wall follow).

Our modes are as follows:
    - 0: direct keyboard teleop (WASD)
    - 1: draw regular polygon
    - 2: letterbox
    - 3: wall follow

The default mode is teleop. From teleop, you can control the neato or
switch into other modes using the following commands:
    - [W][A][S][D]: Manual teleop control
    - [3]-[9]: Draw regular polygons with that many sides
    - [Enter]: Enter Letterbox mode (draw typed word with robot)
    - [K]: Stop robot
    - [Ctrl+C]: Exit the program

While in letterbox, wall follow, or draw shape mode, you can hit any
teleop key [W][A][S][D][K] to re-enter into teleop mode. Additionally,
after the letter(s)/shape(s) are drawn, the neato will return to teleop mode.

If, while drawing a letter or a shape, the neato detects a wall in the way,
the neato will begin to follow that wall until it reaches the end of the wall,
where it will again revert back to teleop.
"""

import rclpy
from rclpy.node import Node
import tty
import tkinter as tk
import select
import sys
import termios
import math
import threading

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from time import sleep
from tkinter import simpledialog
from pyhershey import glyph_factory


class FiniteStateController(Node):
    """
    Teleoperation node with additional autonomous modes.

    Modes:
        - 0: direct keyboard teleop (WASD)
        - 1: draw regular polygon
        - 2: letterbox
        - 3: wall follow

    Publishers:
        - Twist cmd_vel message: Controls robot velocity.

    Subscribers:
        - '/scan' LaserScan scan message: Provides 360° range data.
        - `/odom` provides odometry data with reference to world frame
    """

    def __init__(self):
        """
        Initialize the node and start the keyboard listener.
        """
        super().__init__("fsm_node")
        # Create a timer that fires ten times per second
        timer_period = 0.1

        self.state = 0  # 0 = teleop, 1 = drive shape, 2 = letterbox, 3 = wall follow

        # Run keyboard thread (_keyboard_listener)
        self.settings = termios.tcgetattr(sys.stdin.fileno())
        self.key = None
        self._stop_key_thread = False
        self._key_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._key_thread.start()

        # Run letterbox thread (_draw_letter)
        self._stop_letter_thread = False
        self._letter_thread = threading.Thread(target=self._draw_letter, daemon=True)
        self._letter_thread.start()

        # Run ROS thread on a time to all run_loop
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Odometry, "/odom", self.process_odom, 10)

        # Sensor state
        self.latest_scan = []

        # Initialize odom position and user input
        self.position = [0, 0, 0]
        self.user_input = None

        # General speed params
        self.linear_vel = 0.2
        self.angular_vel = math.pi / 5

        # Drive shape state
        self.segment_num = 1
        self.on_straight = True
        self.segment_start_time = None
        self.num_sides = 4
        self.perimeter = 4
        self.side_len = self.perimeter / self.num_sides

        self.turn_angle = 0
        self.side_time = 0
        self.turn_time = 0

        # Wall follow state
        self.wall_side = "left"
        self.points_per_side = 35
        self.kp = 2.0

        print(
            """
        FSM Teleop Node Initialized!

        Use the following keys to switch modes:

            - [W][A][S][D]: Manual teleop control
            - [3] - [9]: Draw regular polygons with that many sides
            - [Enter]: Enter Letterbox mode (draw typed word with robot)
            - [K]: Stop robot
            - [Ctrl+C]: Exit the program

        Modes:
            0: Teleop
            1: Draw Shape
            2: Letterbox
            3: Wall Follow (Auto-detected when near a wall)

        Switching back to teleop (Mode 0) occurs automatically after tasks or when wall-follow exits.
        """
        )

    def process_scan(self, msg):
        """
        Callback function for /scan subscription.

        Cache the latest LaserScan ranges.

        Args:
            msg (LaserScan): The Odometry message containing pose and orientation.
        """

        self.latest_scan = msg.ranges

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

    def process_key(self):
        """
        Read a key and set state/velocity accordingly.

        Keys:
            w/a/s/d: drive forward/left/back/right
            k: stop
            3 - 9: set polygon sides and start shape mode
            Enter: enter into letterbox mode, take user input
            Ctrl-C: shutdown
        """
        if self.key == "w":
            self.state = 0
            print("Moving forward!")
            self.drive(self.linear_vel, 0.0)

        elif self.key == "a":
            self.state = 0
            print("Moving left!")
            self.drive(0.0, self.angular_vel)

        elif self.key == "s":
            self.state = 0
            print("Moving backward!")
            self.drive(-self.linear_vel, 0.0)

        elif self.key == "d":
            self.state = 0
            print("Moving right!")
            self.drive(0.0, -self.angular_vel)

        elif self.key == "k":
            self.state = 0
            print("Stopping!")
            self.drive(0.0, 0.0)

        elif self.key in ["3", "4", "5", "6", "7", "8", "9"]:
            self._set_draw_shape_params(int(self.key))
            print(f"Drawing shape with {self.key} sides!")
            self.state = 1

        elif self.key == "\n":
            print("Please enter the letter(s) to be drawn!")
            self.state = 2

        elif self.key == "\x03":
            rclpy.shutdown()

        self.key = None

    def drive(self, linear, angular):
        """
        Publish a Twist with the given linear and angular velocity.

        Args:
            linear (float): Linear velocity in m/s.
            angular (float): Angular velocity in rad/s.
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

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
        Main loop to process input and run the active behavior.
        """
        self.process_key()

        if self.state == 1 or self.state == 2:
            self.check_near_wall()

        if self.state == 1:
            self.draw_shape()
        if self.state == 2:
            pass  # handled in draw letterbox thread
        elif self.state == 3:
            self.wall_follow()

    def _draw_letter(self):
        """
        Threaded function that activates when self.state is 2
        Runs in a while loop until then.

        When self.state is 2, it:
        - Prompts user for a word and gets strokes with collect input
        - Moves the robot along the strokes with go_to_point
        - Resets to the home position and sets mode back to teleop.
        """
        while not self._stop_letter_thread:
            # print("Letter thread check!")
            if self.state == 2:
                # print("Letter initatied!")
                strokes_list = self.collect_input()
                if strokes_list:
                    self.go_to_point(0, 0)
                    for character in strokes_list:
                        for segment in character:
                            for point in segment:
                                if self.state == 2:
                                    # print(((point[0])/10, (point[1]/10)))
                                    self.go_to_point((point[0]) / 10, (point[1] / 10))
                        if self.state == 2:
                            self.go_to_point(0, 0)
                    if self.state == 2:
                        self.go_to_point(0, 0)
                        self.state = 0

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
        # print(current_angle, desired_angle)

        current_angle = current_angle % (2 * math.pi)
        desired_angle = desired_angle % (2 * math.pi)

        rotation_needed = (desired_angle - current_angle) % (2 * math.pi)

        angular_vel = 0.5
        lin_velocity = 0.3

        # then we can perform our actual rotation
        if rotation_needed < math.pi and self.state == 2:
            self.drive(linear=0.0, angular=angular_vel)
            sleep((rotation_needed / angular_vel))
            self.drive(linear=0.0, angular=0.0)
        elif self.state == 2:
            rotation_needed = 2 * math.pi - rotation_needed
            self.drive(linear=0.0, angular=-angular_vel)
            sleep((rotation_needed / angular_vel))
            self.drive(linear=0.0, angular=0.0)

        if self.state == 2:
            # calculate needed distance and drive forward
            distance = math.sqrt((x) ** 2 + (y) ** 2)
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
                    strokes.append(
                        glyph_factory.from_ascii(character, "roman_simplex").segments
                    )
                except:
                    print("Invalid Character(s)!")
            return strokes
        else:
            self.state = 0
            print("User cancelled.")
            return []

    def wall_follow(self):
        """
        Follow a nearby wall using proportional control on range error.
        """
        self.find_wall_side()

        if self.wall_side == "left":
            index_1 = 90 - self.points_per_side
            index_2 = 90 + self.points_per_side
        else:
            index_1 = 270 - self.points_per_side
            index_2 = 270 + self.points_per_side

        error = 0
        num_counted = 0
        for i in range(self.points_per_side):
            if (
                self.latest_scan[index_1 + i] != 0.0
                and self.latest_scan[index_2 - i] != 0.0
            ):
                error += self.latest_scan[index_2 - i] - self.latest_scan[index_1 + i]
                num_counted += 1

        if num_counted != 0:
            error /= num_counted

        if 0 < math.fabs(error) < 0.005:
            if self.check_wall_in_front():
                print("Wall detected in front, stopping and entering teleop")
                self.drive(0.0, 0.0)
                self.state = 0
                return

        # print(f"Error {error}")
        angular = -self.kp * float(error)

        if math.fabs(error) > 0.01:
            self.drive(0.0, angular)
        else:
            self.drive(0.1, angular)

    def draw_shape(self):
        """
        Drive a regular polygon by alternating straight segments and turns.
        """
        if self.segment_start_time is None:
            self.segment_start_time = self.get_clock().now()

        if self.on_straight:
            seg_time = self.side_time
        else:
            seg_time = self.turn_time

        # print(
        #     f"Time: {self.get_clock().now() - self.segment_start_time}, {rclpy.time.Duration(seconds=seg_time)}"
        # )

        if self.get_clock().now() - self.segment_start_time >= rclpy.time.Duration(
            seconds=seg_time
        ):
            if not self.on_straight:
                print(f"Finished segment and turn {self.segment_num}")
                self.segment_num += 1
                if self.segment_num > self.num_sides:
                    self.state = 0
                    self.drive(0.0, 0.0)
                    return

            self.on_straight = not self.on_straight
            self.segment_start_time = None
        else:
            if self.on_straight:
                self.drive(self.linear_vel, 0.0)
            else:
                self.drive(0.0, -self.angular_vel)

    def check_near_wall(self):
        """
        Detect proximity to walls and switch to wall-follow mode if close.
        """
        scan = self.latest_scan

        left_avg = self._average_of_range(scan, 25, 35)
        right_avg = self._average_of_range(scan, 325, 335)
        center_avg = (
            self._average_of_range(scan, 355, 360) + self._average_of_range(scan, 0, 5)
        ) / 2

        threshold = 0.8
        if (
            int(left_avg < threshold * 1.2)
            + int(right_avg < threshold * 1.2)
            + int(center_avg < threshold)
        ) >= 2:
            self.state = 3
            print("Wall Detected. Ending letter/shape, starting wall follow")

    def _average_of_range(self, scan, start_index, end_index):
        """
        Compute the average of valid scan values between two indices.

        Args:
            scan (list[float]): Full scan array.
            start_index (int): Inclusive start index.
            end_index (int): Inclusive end index.

        Returns:
            float: Mean distance over valid points, or 2 if none valid.
        """
        dist = 0
        num_points = 0

        for i in range(start_index, end_index + 1):
            if scan[i] > 0.1:
                dist += scan[i]
                num_points += 1

        if num_points > 0:
            return dist / num_points

        return 2

    def check_wall_in_front(self):
        """
        Check if an obstacle is within a forward threshold.

        Returns:
            bool: True if a wall is closer than 0.7 m ahead.
        """
        min_dist = 100

        for i in range(357, 360):
            if self.latest_scan[i] > 0.1:
                min_dist = min(min_dist, self.latest_scan[i])

        if min_dist < 0.7:
            return True

        return False

    def find_wall_side(self):
        """
        Determine which side has more points within 1 m and set wall_side.
        """
        scan_ranges = self.latest_scan
        max_dist = 1
        min_dist = 0.1
        left_points = 0
        right_points = 0
        for i in range(180):
            if min_dist < scan_ranges[i] < max_dist:
                left_points += 1
            if min_dist < scan_ranges[i + 180] < max_dist:
                right_points += 1

        if left_points > right_points:
            self.wall_side = "left"
            # print("Wall on left")
        else:
            self.wall_side = "right"
            # print("Wall on right")

    def _set_draw_shape_params(self, num_sides):
        """
        Set polygon parameters and derived timings for shape mode.
        """
        self.num_sides = num_sides
        self.side_len = self.perimeter / self.num_sides
        sum_of_angles = (self.num_sides - 2) * math.radians(180)
        self.turn_angle = math.radians(180) - sum_of_angles / self.num_sides
        self.side_time = self.side_len / self.linear_vel
        self.turn_time = self.turn_angle / self.angular_vel

        # Define where we are in shape
        self.segment_num = 1
        self.on_straight = True
        self.segment_start_time = None


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
    rclpy.init(args=args)  # Initialize communication with ROS
    node = FiniteStateController()

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


if __name__ == "__main__":
    main()

"""This is a ROS node that uses python and ROS to teleop into
the robot."""

import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios
import math
import threading

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FiniteStateController(Node):
    """This is a teloperation node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the TeleopNode node. No inputs."""
        super().__init__("teleop_node")
        # Create a timer that fires ten times per second
        timer_period = 0.1

        # Run keyboard thread
        self.settings = termios.tcgetattr(sys.stdin.fileno())
        self.key = None
        self._key_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._key_thread.start()

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.state = 0  # 0 = teleop, 1 = drive shape, 2 = letterbox, 3 = wall follow

        # Sensor state
        self.latest_scan = []

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
        self.kp = 20.0

    def process_scan(self, msg):
        self.latest_scan = msg.ranges

    def process_key(self):
        """Takes key input and drives in a direction according to the input
        Legend:
        w - forward
        a - left
        s - backwards
        d - right
        any other key - stop"""

        if self.key == "w":
            self.state = 0
            self.drive(self.linear_vel, 0.0)

        elif self.key == "a":
            self.state = 0
            self.drive(0.0, self.angular_vel)

        elif self.key == "s":
            self.state = 0
            self.drive(-self.linear_vel, 0.0)

        elif self.key == "d":
            self.state = 0
            self.drive(0.0, -self.angular_vel)

        elif self.key == "k":
            self.state = 0
            self.drive(0.0, 0.0)

        elif self.key in ["3", "4", "5", "6", "7", "8", "9"]:
            self._set_draw_shape_params(int(self.key))
            self.state = 1

        elif self.key == "\x03":
            rclpy.shutdown()

        self.key = None

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

    def _keyboard_listener(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self.key = sys.stdin.read(1)
                    # Debug:
                    print(f"Key captured in thread: {repr(self.key)}", flush=True)
        finally:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin)
            )

    def run_loop(self):
        """Prints a message to the terminal."""
        self.process_key()

        if self.state == 1 or self.state == 2:
            self.check_near_wall()

        if self.state == 1:
            self.draw_shape()
        if self.state == 2:
            self.letterbox()
        elif self.state == 3:
            self.wall_follow()

    def letterbox(self):
        pass

    def wall_follow(self):
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
                error = self.latest_scan[index_2] - self.latest_scan[index_1]
                num_counted += 1

        if num_counted != 0:
            error /= num_counted

        if 0 < math.fabs(error) < 0.005:
            if self.check_wall_in_front():
                print("Wall detected in front, stopping and entering teleop")
                self.drive(0.0, 0.0)
                self.state = 0
                return

        angular = -self.kp * float(error)

        if math.fabs(error) > 0.01:
            self.drive(0.0, angular)
        else:
            self.drive(0.1, angular)

    def draw_shape(self):
        if self.segment_start_time is None:
            self.segment_start_time = self.get_clock().now()

        if self.on_straight:
            seg_time = self.side_time
        else:
            seg_time = self.turn_time

        print(
            f"Time: {self.get_clock().now() - self.segment_start_time}, {rclpy.time.Duration(seconds=seg_time)}"
        )

        if self.get_clock().now() - self.segment_start_time >= rclpy.time.Duration(
            seconds=seg_time
        ):
            if not self.on_straight:
                print(f"Finished segment and turn {self.segment_num}")
                self.segment_num += 1
                if self.segment_num > self.num_sides:
                    self.state = 0
                    return

            self.on_straight = not self.on_straight
            self.segment_start_time = None
        else:
            if self.on_straight:
                self.drive(self.linear_vel, 0.0)
            else:
                self.drive(0.0, -self.angular_vel)

    def check_near_wall(self):
        scan = self.latest_scan

        left_avg = self._average_of_range(scan, 15, 30)
        right_avg = self._average_of_range(scan, 330, 345)
        center_avg = (
            self._average_of_range(scan, 355, 360) + self._average_of_range(scan, 0, 5)
        ) / 2

        threshold = 0.6
        if (
            int(left_avg < threshold * 1.2)
            + int(right_avg < threshold * 1.2)
            + int(center_avg < threshold)
        ) >= 2:
            self.state = 3
            print("Wall Detected. Ending letter/shape, starting wall follow")

    def _average_of_range(self, scan, start_index, end_index):
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

        min_dist = 100

        for i in range(357, 360):
            if self.latest_scan[i] > 0.1:
                min_dist = min(min_dist, self.latest_scan[i])

        if min_dist < 0.7:
            return True

        return False

    def find_wall_side(self):
        """
        Sense which side the wall is on from lidar scan.

        The side with more points within 1 meter is determined to be wall side.
        Always assumes wall is present.

        Args:
            scan_ranges (list of ints): Ranges from scan to process
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
            print("Wall on left")
        else:
            self.wall_side = "right"
            print("Wall on right")

    def _set_draw_shape_params(self, num_sides):
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


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = FiniteStateController()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()

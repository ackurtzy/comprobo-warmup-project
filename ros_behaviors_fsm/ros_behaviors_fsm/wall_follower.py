"""
This node uses the lidar to locate which side a wall is on, then drives
parallel using proportional control.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class WallApproachNode(Node):
    """
    This is a node which uses lidar to sense wall and drive parallel to it.

    It always assumes there is a wall to follow and senses which side the is on
    by assuming it is the side with more points within 1 meter. It
    uses proportional control to maintain parallel heading. The proportional
    constant can be adjusted with the ROS param kp.

    Publishers:
        - Twist to cmd_vel topic: Controls Neato velocity.
    Subscribers:
        - LaserScan from scan topic: Lidar scan for wall sensing
    """

    def __init__(self):
        """
        Initialize the node.
        """
        super().__init__("wall_follower")
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.error = None
        self.wall_side = None
        self.points_per_side = 45

        # the following three lines are needed to support parameters
        self.declare_parameters(namespace="", parameters=[("kp", 2.0)])
        self.kp = self.get_parameter("kp").value
        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Set the proportional constant dynamically"""
        for param in params:
            if param.name == "kp" and param.type_ == Parameter.Type.DOUBLE:
                self.kp = param.value
        print(self.kp)
        return SetParametersResult(successful=True)

    def run_loop(self):
        """
        Main drive loop.

        Publishes angular velocity message every run that is calculated using
        error and kp.
        """
        msg = Twist()
        msg.linear.x = 0.1
        if self.error is not None:
            # use proportional control to set the angular velocity
            msg.angular.z = -self.kp * float(self.error)
        self.vel_pub.publish(msg)

    def find_wall_side(self, scan_ranges):
        """
        Sense which side the wall is on from lidar scan.

        The side with more points within 1 meter is determined to be wall side.
        Always assumes wall is present.

        Args:
            scan_ranges (list of ints): Ranges from scan to process
        """
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

    def process_scan(self, msg):
        """
        Process the lidar scan to update error.

        See README for error methodology explanation.

        Args:
            msg (Laserscan): Laserscan message from subscriber
        """
        self.find_wall_side(msg.ranges)

        if self.wall_side == "left":
            index_1 = 90 - self.points_per_side
            index_2 = 90 + self.points_per_side
        else:
            index_1 = 270 - self.points_per_side
            index_2 = 270 + self.points_per_side

        self.error = 0
        num_counted = 0
        for i in range(self.points_per_side):
            if msg.ranges[index_1 + i] != 0.0 and msg.ranges[index_2 - i] != 0.0:
                self.error += msg.ranges[index_2 - i] - msg.ranges[index_1 + i]
                num_counted += 1

        if num_counted != 0:
            self.error /= num_counted


def main(args=None):
    """
    Intitialize rclpy and Node, then run.
    """
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

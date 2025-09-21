"""This node uses the laser scan measurement pointing straight ahead from
the robot and compares it to a desired set distance.  The forward velocity
of the robot is adjusted until the robot achieves the desired distance"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
import math


class WallApproachNode(Node):
    """This class wraps the basic functionality of the node"""

    def __init__(self):
        super().__init__("wall_follower")
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.angle_difference = None
        self.wall_side = None
        self.points_per_side = 45

        # the following three lines are needed to support parameters
        self.declare_parameters(
            namespace="", parameters=[("Kp", 15), ("target_distance", 0.5)]
        )
        self.Kp = self.get_parameter("Kp").value
        self.target_distance = self.get_parameter("target_distance").value
        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """This callback allows the parameters of the node to be adjusted
        dynamically by other ROS nodes (e.g., dynamic_reconfigure).
        This function is only needed to support dynamic_reconfigure."""
        for param in params:
            if param.name == "Kp" and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
            elif (
                param.name == "target_distance" and param.type_ == Parameter.Type.DOUBLE
            ):
                self.target_distance = param.value
        print(self.Kp, self.target_distance)
        return SetParametersResult(successful=True)

    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0.1
        if self.angle_difference is not None:
            # use proportional control to set the angular velocity
            msg.angular.z = -self.Kp * float(self.angle_difference)
        self.vel_pub.publish(msg)

    def find_wall_side(self, scan_ranges):
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
        # if not self.wall_side:
        self.find_wall_side(msg.ranges)

        if self.wall_side == "left":
            index_1 = 90 - self.points_per_side
            index_2 = 90 + self.points_per_side
        else:
            index_1 = 270 - self.points_per_side
            index_2 = 270 + self.points_per_side

        self.angle_difference = 0
        num_counted = 0
        for i in range(self.points_per_side):
            if msg.ranges[index_1 + i] != 0.0 and msg.ranges[index_2 - i] != 0.0:
                self.angle_difference = msg.ranges[index_2] - msg.ranges[index_1]
                num_counted += 1

        if num_counted != 0:
            self.angle_difference /= num_counted


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

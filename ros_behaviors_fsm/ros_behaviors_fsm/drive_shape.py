"""A node the drives the Neato in an arbitrary polygon"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class DriveShapeNode(Node):
    """
    This is a node which drives the robot in a regular polygon defined by params
    side_len and num_sides.

    The Neato will move forward side_len meters and turn clockwise the exterior
    angle of a shape with num_sides until it has completed num_sides segments.

    Publishers:
        - Twist cmd_vel message: Controls neato velocity.
    """

    def __init__(self):
        """
        Initialize the node. By default, will be a 1 meter square.
        """
        super().__init__("drive_square")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Define speed parameters
        self.linear_vel = 0.2
        self.angular_vel = math.pi / 8

        # Define shape parameters
        self.declare_parameters(
            namespace="", parameters=[("side_len", 1.0), ("num_sides", 4)]
        )
        self.side_len = self.get_parameter("side_len").value
        self.num_sides = self.get_parameter("num_sides").value

        # Need at least 3 sides
        if self.num_sides < 3:
            raise ValueError("num_sides must be >= 3")

        # Derive angle and segment time accordingly
        sum_of_angles = (self.num_sides - 2) * math.radians(180)
        self.turn_angle = math.radians(180) - sum_of_angles / self.num_sides
        self.side_time = self.side_len / self.linear_vel
        self.turn_time = self.turn_angle / self.angular_vel

        # Define where we are in shape
        self.segment_num = 1
        self.on_straight = True
        self.segment_start_time = None

        self.add_on_set_parameters_callback(self.parameter_callback)

        print("Starting to drive")

    def parameter_callback(self, params):
        """
        Enable dynamically reconfiguring the shape's side_len and num_sides.
        If robot is mid-shape, will start new shape in place.
        """

        for param in params:
            if param.name == "side_len" and param.type_ == Parameter.Type.DOUBLE:
                self.side_len = param.value
            elif param.name == "num_sides" and param.type_ == Parameter.Type.INTEGER:
                self.num_sides = param.value
                if self.num_sides < 3:
                    return SetParametersResult(successful=True)

        # Recalculate derived turn angle and segment time
        sum_of_angles = (self.num_sides - 2) * math.radians(180)
        self.turn_angle = math.radians(180) - sum_of_angles / self.num_sides
        self.side_time = self.side_len / self.linear_vel
        self.turn_time = self.turn_angle / self.angular_vel

        # Restart shape
        self.segment_num = 1
        self.on_straight = True
        self.segment_start_time = None

        print(
            f"Shape restarting, updated shape parameters: {self.num_sides} sides of length {self.side_len} m"
        )

        return SetParametersResult(successful=True)

    def run_loop(self):
        """
        Main drive loop. Publishes velocity message every run.

        Uses Node's built in get_clock() for simulation speed change compatibility.
        """
        vel_msg = Twist()

        if self.segment_start_time is None:
            self.segment_start_time = self.get_clock().now()

        if self.on_straight:
            seg_time = self.side_time
        else:
            seg_time = self.turn_time

        if self.get_clock().now() - self.segment_start_time >= rclpy.time.Duration(
            seconds=seg_time
        ):
            if not self.on_straight:
                print(f"Finished segment and turn {self.segment_num}")
                self.segment_num += 1
                if self.segment_num > self.num_sides:
                    self.vel_pub.publish(vel_msg)
                    self.destroy_node()
                    rclpy.shutdown()

            self.on_straight = not self.on_straight
            self.segment_start_time = None
        else:
            if self.on_straight:
                vel_msg.linear.x = self.linear_vel
            else:
                vel_msg.angular.z = -self.angular_vel

        self.vel_pub.publish(vel_msg)


def main(args=None):
    """
    Intitialize rclpy and Node, then run.
    """
    rclpy.init(args=args)
    node = DriveShapeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

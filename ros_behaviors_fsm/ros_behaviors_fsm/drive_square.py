import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class DriveShapeNode(Node):

    def __init__(self):
        super().__init__("drive_square")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Define speed parameters
        self.linear_vel = 0.2
        self.angular_vel = math.pi / 8

        # Define shape parameters
        self.side_len = 0.8
        self.turn_angle = math.radians(72)  # math.pi / 2
        self.side_time = self.side_len / self.linear_vel
        self.turn_time = self.turn_angle / self.angular_vel
        self.num_sides = 5

        # Define where we are in shape
        self.segment_num = 1
        self.on_straight = True
        self.segment_start_time = None

        print(f"Starting to drive")

    def run_loop(self):
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

        print(vel_msg)
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveShapeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

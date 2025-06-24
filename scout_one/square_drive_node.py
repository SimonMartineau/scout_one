#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def seconds_to_duration(seconds: float):
    """
    Helper to convert seconds to ROS2 Duration-like float for internal timing.
    """
    return seconds

class SquareDrive(Node):
    def __init__(self):
        super().__init__('square_drive')
        # Publisher to cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters for square path
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('side_length', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.side_length  = self.get_parameter('side_length').value

        # Compute durations
        self.linear_duration = self.side_length / self.linear_speed
        self.turn_duration   = (3.14159265 / 2) / self.angular_speed

        # State machine
        self.state = 0  # 0: driving straight, 1: turning
        self.start_time = self.get_clock().now()

        # Timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('SquareDrive node started: driving square with side_length=%.2f m' % self.side_length)

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        twist = Twist()

        if self.state == 0:
            # Drive straight
            if elapsed < self.linear_duration:
                twist.linear.x = self.linear_speed
            else:
                # Switch to turning
                self.state = 1
                self.start_time = now
                self.get_logger().info('Starting turn')
                return

        elif self.state == 1:
            # Turn
            if elapsed < self.turn_duration:
                twist.angular.z = self.angular_speed
            else:
                # Completed one side + turn, switch back to straight
                self.state = 0
                self.start_time = now
                self.get_logger().info('Starting straight segment')
                return

        # Publish command
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down')
    finally:
        # Stop the robot
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

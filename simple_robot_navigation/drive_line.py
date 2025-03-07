import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to send velocity commands
        timer_period = 0.1  # seconds per callback
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # For Odometry
        self.start_x = None # init x
        self.start_y = None # init y
        self.target_distance = 2.0 # move this far in a straight line

        # Starts at max speed, will gradually decrease near target
        self.current_speed = 0.1

        # Subscribe to odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # How far the robot has moved from start pos 
        # x and y components
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_x == None:
            self.start_x = x
            self.start_y = y
        
        # Total distance moved from start
        dist_moved = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
        remaining = self.target_distance - dist_moved

        # Stop if target has been reached
        if remaining <= 0:
            # 0 velocity twist
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)

            # Destroy timer
            self.destroy_timer(self.timer)
        # Start slowing down if near target
        elif remaining <= 0.5:
            # Minimum speed is 0.01 so target is actually reached
            self.current_speed = max(0.01, 0.1 * remaining)


    def timer_callback(self):
        msg = Twist()

        msg.linear.x = self.current_speed
        self.publisher_.publish(msg)
        print("Publishing\n")

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
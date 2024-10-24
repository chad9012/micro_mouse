#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        
        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscribers for the ultrasonic sensor topics
        self.create_subscription(Range, '/front_ultrasonic_sensor_range', self.front_sensor_callback, 10)
        self.create_subscription(Range, '/left_ultrasonic_sensor_range', self.left_sensor_callback, 10)
        self.create_subscription(Range, '/right_ultrasonic_sensor_range', self.right_sensor_callback, 10)
        self.create_subscription(Range, '/rear_ultrasonic_sensor_range', self.rear_sensor_callback, 10)

        # Initialize sensor data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.rear_distance = float('inf')

        # Define the movement control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def front_sensor_callback(self, msg):
        self.front_distance = msg.range

    def left_sensor_callback(self, msg):
        self.left_distance = msg.range

    def right_sensor_callback(self, msg):
        self.right_distance = msg.range

    def rear_sensor_callback(self, msg):
        self.rear_distance = msg.range

    def control_loop(self):
        twist_msg = Twist()

        # Basic maze-solving logic
        # Move forward if there's no obstacle in front
        if self.front_distance > 0.5:  # Adjust threshold as needed
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0  # No rotation
        else:
            # Turn based on the sensor readings
            if self.left_distance > self.right_distance:
                twist_msg.linear.x = 0.0  # Stop forward movement
                twist_msg.angular.z = 0.5  # Turn left
            else:
                twist_msg.linear.x = 0.0  # Stop forward movement
                twist_msg.angular.z = -0.5  # Turn right

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

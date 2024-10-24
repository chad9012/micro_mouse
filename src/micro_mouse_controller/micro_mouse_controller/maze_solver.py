#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String  # To subscribe to the robot's area info

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

        # Subscribe to the topic that publishes the current area
        self.create_subscription(String, '/robot_area', self.area_callback, 10)

        # Initialize sensor data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.rear_distance = float('inf')

        # Initialize area data
        self.current_area = None

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

    def area_callback(self, msg):
        """Callback function to update the robot's current area."""
        self.current_area = msg.data  # Update the area (should be "1", "2", or "3")
        self.get_logger().info(f"Current area: {self.current_area}")

    def control_loop(self):
        """Control logic for navigating through the maze."""
        twist_msg = Twist()

        # Check if the robot has reached Area 3 or Area 1
        if self.current_area == "3":
            self.get_logger().info("Robot has reached the final position (Area 3), stopping.")
            twist_msg.linear.x = 0.0  # Stop the robot
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            return

        if self.current_area == "1" and self.front_distance < 0.5:
            self.get_logger().info("Robot is back at the start position (Area 1), stopping.")
            twist_msg.linear.x = 0.0  # Stop the robot
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            return

        # Example movement logic: Simple obstacle avoidance
        if self.front_distance < 0.5:
            # If an obstacle is detected in front, turn right
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.5
        else:
            # Otherwise, move forward
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0

        # Publish the movement command
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

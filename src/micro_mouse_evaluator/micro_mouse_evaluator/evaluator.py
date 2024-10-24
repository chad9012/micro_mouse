import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # Import the message type for publishing the area
import time
from datetime import datetime

class Evaluator(Node):
    def __init__(self, name, roll_no, branch):
        super().__init__('micro_mouse_evaluator')

        # Subscription to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Change to your actual odometry topic
            self.odom_callback,
            10
        )

        # Publisher to publish the robot's current area
        self.area_publisher = self.create_publisher(String, '/robot_area', 10)

        self.reset_timer()
        self.start_position = (-4.5, -4.5, 0.0)
        self.final_position = (4.5, 4.5, 0.0)
        self.best_time = float('inf')
        self.run_count = 0
        self.previous_area = None  # Track the previous area
        self.current_area = None  # Track the current area

        self.noise_margin = 0.5  # Margin for noise in positions

        # User information
        self.name = name
        self.roll_no = roll_no
        self.branch = branch

    def reset_timer(self):
        self.start_time = None
        self.elapsed_time = 0.0
        self.get_logger().info("Timer reset.")

    def get_area(self, current_position):
        """Determine which area the robot is currently in."""
        if (self.start_position[0] - self.noise_margin <= current_position[0] <= self.start_position[0] + self.noise_margin and
            self.start_position[1] - self.noise_margin <= current_position[1] <= self.start_position[1] + self.noise_margin):
            return 1  # Area 1: Start position area

        elif (self.final_position[0] - self.noise_margin <= current_position[0] <= self.final_position[0] + self.noise_margin and
              self.final_position[1] - self.noise_margin <= current_position[1] <= self.final_position[1] + self.noise_margin):
            return 3  # Area 3: Final position area

        else:
            return 2  # Area 2: Everything else

    def publish_area(self, area):
        """Publish the current area as a string."""
        area_msg = String()

        if area == 1:
            area_msg.data = "1"
        elif area == 2:
            area_msg.data = "2"
        elif area == 3:
            area_msg.data = "3"

        # Publish the message to the topic
        self.area_publisher.publish(area_msg)
        self.get_logger().info(f"Published: {area_msg.data}")

    def odom_callback(self, msg):
        # Get the current position from odometry
        current_position = (msg.pose.pose.position.x, 
                            msg.pose.pose.position.y, 
                            msg.pose.pose.position.z)

        self.current_area = self.get_area(current_position)

        # Only proceed if the area has changed
        if self.current_area == self.previous_area:
            return  # Do nothing if the area hasn't changed

        # Publish the current area
        self.publish_area(self.current_area)

        # Start the timer when transitioning from Area 1 to Area 2
        if self.previous_area == 1 and self.current_area == 2:
            if self.start_time is None:
                self.start_time = time.time()  # Start the timer
                self.get_logger().info("Timer started.")

        # Stop the timer when transitioning from Area 2 to Area 3
        elif self.previous_area == 2 and self.current_area == 3:
            if self.start_time is not None:
                self.elapsed_time = time.time() - self.start_time
                self.get_logger().info(f'Run finished in {self.elapsed_time:.2f} seconds.')
                self.start_time = None  # Stop the timer

                self.run_count += 1
                self.best_time = min(self.best_time, self.elapsed_time)
                
                if self.run_count < 3:
                    self.get_logger().info(f'Best time so far: {self.best_time:.2f} seconds over {self.run_count} runs.')
                else:
                    self.get_logger().info(f'Final best time: {self.best_time:.2f} seconds after 3 runs.')
                    self.write_submission_file()  # Write the submission file
                    self.reset_timer()  # Reset for next set of runs
                    self.run_count = 0
                    self.best_time = float('inf')

        # Update the previous area
        self.previous_area = self.current_area

    def write_submission_file(self):
        """Write the submission details to a text file."""
        with open('submission.txt', 'w') as f:
            f.write(f"Name: {self.name}\n")
            f.write(f"Roll No: {self.roll_no}\n")
            f.write(f"Branch: {self.branch}\n")
            f.write(f"Best Time: {self.best_time:.2f} seconds\n")
            f.write(f"Date and Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        self.get_logger().info("Submission file written.")

def main(args=None):
    rclpy.init(args=args)

    # Replace with actual user details
    evaluator = Evaluator("Your Name", "Your Roll No", "Your Branch")

    rclpy.spin(evaluator)

    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

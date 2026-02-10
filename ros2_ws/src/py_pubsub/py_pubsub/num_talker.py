# Author: Milton Tinoco 
# File Name: num_talker.py
# Description: This file defines a ros2 node that publishes random integers 
# to a topic named 'topic_number_hw1' at regular intervals.

# Import necessary libraries

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Define the NumberTalker class that inherits from Node
class NumberTalker(Node):
    
    # Initialize the NumberTalker node and create a publisher to the 'topic_number_hw1' topic
    def __init__(self):
        super().__init__('num_talker') # Initialize the node with the name 'num_talker'
        self.publisher_ = self.create_publisher(Int32, 'topic_number_hw1', 10) # Create a publisher that will publish Int32 messages to the 'topic_number_hw1' topic with a queue size of 10
        timer_period = .5  # Set the timer period to 0.5 seconds (500 milliseconds)
        self.timer = self.create_timer(timer_period, self.timer_callback) # Create a timer that will call the timer_callback function every 0.5 seconds

    # Define the timer callback function that will be called every time the timer expires
    # This function generates a random integer between 0 and 100, publishes it to the 'topic_number_hw1' topic, and logs the published number
    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(0, 100)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing number: "%d"' % msg.data)

# Define the main function that initializes the ROS2 node and starts spinning to publish messages

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS2 Python client library
    number_talker = NumberTalker() # Create an instance of the NumberTalker class, which initializes the node and sets up the publisher and timer
    rclpy.spin(number_talker) # Keep the node running and publishing messages until it is shut down
    number_talker.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shut down the ROS2 client library

if __name__ == '__main__':
    main()

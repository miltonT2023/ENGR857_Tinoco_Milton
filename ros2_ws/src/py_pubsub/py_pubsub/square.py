# Author: Milton Tinoco 
# File Name: square.py
# Description: This file defines a ros2 node that subscribes to a topic named 'topic_number_hw1' and listens for Int32 messages.
# Upon receiving a message, it calculates the square of the received number and print the both the original number and its squared value.

# Import necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Define the Square class that inherits from Node
class Square(Node):
    
    # Initialize the Square node and create a subscription to the 'topic_number_hw1' topic
    def __init__(self):
        super().__init__('square')

        # Create a subscription to the 'topic_number_hw1' topic, which expects Int32 messages. The callback function is listener_callback.
        self.subscription = self.create_subscription(
            Int32,
            'topic_number_hw1',
            self.listener_callback,
            10)
        self.subscription 

    # Define the callback function that will be called when a message is received on the 'topic_number_hw1' topic
    def listener_callback(self, msg):
        squared_values = msg.data * msg.data
        self.get_logger().info('I heard "%d", Squared: "%d"' % (msg.data, squared_values))


# Define the main function that initializes the ROS2 node and starts spinning to listen for messages
def main(args=None):
    rclpy.init(args=args) # Initialize the ROS2 Python client library
    square = Square() # Create an instance of the Square class, which initializes the node and sets up the subscription
    rclpy.spin(square) # Keep the node running and listening for messages until it is shut down
    square.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shut down the ROS2 client library


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# Install the geographiclib 2.0 module for this code to work.
# To install geographiclib 2.0, copy the line below to your terminal.
# pip install geographiclib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from parc_robot_bringup.gps2cartesian import gps_to_cartesian

class Example1(Node):
    def __init__(self):
        super().__init__("example1_basic")

        # Subscribe to the GPS topic - ### SENSING
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 1)

        # Publish to /robot_base_controller/cmd_vel_unstamped
        self.pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)

        # Robot start position
        self.robot_start_x = 8.850000
        self.robot_start_y = -0.031845

    # GPS callback function
    def gps_callback(self, gps):

        #### THINK

        # Get the cartesian coordinates from the GPS coordinates
        current_x, current_y = gps_to_cartesian(gps.latitude, gps.longitude)      

        # Create a Twist message for linear x and angular z values
        move_cmd = Twist()

        # Check if the absolute value between the robot start x position and its current
        # x position is less than 6 metres
        if abs(self.robot_start_x - current_x) < 6.0:
            move_cmd.linear.x = 0.25  # move in x axis at 0.25 m/s
            move_cmd.angular.z = 0.0
        else:
            # Stop the robot
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        
        ### ACT

        # Publish message to the robot
        self.pub.publish(move_cmd)

# Main function
def main(args=None):

    # Initialize ROS library
    rclpy.init(args=args)

    # Create node
    example1_full_node = Example1()

    # Spin the node
    rclpy.spin(example1_full_node)

    example1_full_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()







#!/usr/bin/env python

"""
@file       poseHub.py
@author     Team 3 (Muhammad Fazrey Bin Zainal, Kaushik Thirumavalavan, Akmal Rusyaidi Bin Muhammad Tajwid,
            Muhammad Zulhilman Bin Mohd Shaini, Chang Keng Jethro, Janaishwaran)
@course     Robotics Systems Engineering, Singapore Institute of Technology
@module     SEP
@date       24/07/2023
@brief      This script allows the user to control the pose estimate of the robot through the terimnal using WASDJK controls

"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import roslib
import sys, select, termios, tty

#CONFIGS
wasd = 0.02
jk = 0.02

# Initialize the ROS node with the name 'pose_publisher'
rospy.init_node('pose_publisher')

# Create a publisher to publish the initial pose on the topic '/initialpose'
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

# Global variable to store the current pose
curr_pose = None

# Callback function to update the current pose when new data is received on '/amcl_pose' topic	
def pose_callback(pose_msg):
    global curr_pose
    curr_pose = pose_msg

# Function to read a single key from the keyboard without the need to press enter to register input
# Function to read a single key from the keyboard without the need to press enter to register input
def getKey(key_timeout):
    # Get the current terminal settings to be restored later
    settings = termios.tcgetattr(sys.stdin)
    
    # Set the terminal to raw mode, which allows reading a single character at a time without buffering
    tty.setraw(sys.stdin.fileno())

    # Use the select module to check for keyboard input with a specified timeout
    # rlist will contain the file objects that are ready to be read
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)

    # Check if there is any keyboard input available within the specified timeout
    if rlist:
        # Read a single character from the standard input (stdin)
        key = sys.stdin.read(1)
    else:
        # If there is no input available within the timeout, set the key to an empty string
        key = ''

    # Restore the original terminal settings to allow normal terminal behavior
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # Return the single character read from the keyboard or an empty string if no input is available
    return key

# Subscriber function to subscribe to the '/amcl_pose' topic and update the current pose
def subscriber():
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

# Main loop to continuously update the pose and publish it when a key is pressed
print("position: wasd, orientation: jk, 'p' to exit")
while True:
    subscriber()  # Update current position
    if curr_pose is not None:
        pose_msg = curr_pose
        key_timeout = rospy.get_param("~key_timeout", 0.0)
        val = getKey(key_timeout)
        wasd = 0.1  # Translation increment value
        jk = 0.1    # Orientation increment value

        # Check which key is pressed and modify the pose accordingly
        if val == 'w':
            print('w\n')
            pose_msg.pose.pose.position.x += wasd  # Increment x position
        elif val == 'a':
            print('a\n')
            pose_msg.pose.pose.position.y += wasd  # Increment y position
        elif val == 's':
            print('s\n')
            pose_msg.pose.pose.position.x -= wasd  # Decrement x position
        elif val == 'd':
            print('d\n')
            pose_msg.pose.pose.position.y -= wasd  # Decrement y position
        elif val == 'j':
            print('j\n')
            pose_msg.pose.pose.orientation.z += jk  # Increment orientation (z-axis)
        elif val == 'k':
            print('k\n')
            pose_msg.pose.pose.orientation.z -= jk  # Decrement orientation (z-axis)
        elif val == 'p':
            print("exiting...")  # Print message when 'p' key is pressed
            break  # Exit the loop and end the program

        # Publish the modified pose
        pub.publish(pose_msg)

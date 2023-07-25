#!/usr/bin/env python

"""
@file       egui.py
@author     Team 3 (Muhammad Fazrey Bin Zainal, Kaushik Thirumavalavan, Akmal Rusyaidi Bin Muhammad Tajwid,
            Muhammad Zulhilman Bin Mohd Shaini, Chang Keng Jethro, Janaishwaran)
@course     Robotics Systems Engineering, Singapore Institute of Technology
@module     SEP
@date       24/07/2023
@brief      This script retrieves information from the /amcl_pose and /odom topic and converts the position 
            and orientation information into a usable array format that can be directly use in the coordinate base
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
val = 0

# Node initialization
rospy.init_node('init_pose')

# Function to extract x, y, z, and w values from a given text
def extractor(text):
    values = []
    lines = text.strip().split('\n')

    for line in lines:
        if ':' in line:
            key, value = line.split(':') #finds messages in the text with a colon and converts them into key and value pair
            key = key.strip() #cleans the string by removing white space
            value = value.strip() #cleans the string by removing white space
            if key in ['x', 'y', 'z', 'w']: #checks if the line contains coordinate information
                if value:
                    try:
                        value = float(value)
                        values.append(value)
                    except ValueError:
                        pass

    print(values)

def get_odom():
    # Get Odometry pose from Gazebo
  odom_msg = rospy.wait_for_message('/odom', Odometry)

  print("ODOM------------------------------------------------------/")
  # Print the receives message in array format
  extractor(str(odom_msg))

  # Return the pose message
  return odom_msg


def get_amcl():
    # Get amcl pose from Gazebo
  amcl_pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
  
  print("AMCL------------------------------------------------------/")
  # Print the received pose message in array format
  extractor(str(amcl_pose_msg))

  # Return the pose message
  return amcl_pose_msg    
  
# Keeps looping for input, gets amcl coords if 3 is input, odom coords if 2 is input
while True:
  val = input("enter 2 for /odom and 3 for /amcl_pose for coords: ")
  if val == 2:
    get_odom()
  if val == 3:
    get_amcl()

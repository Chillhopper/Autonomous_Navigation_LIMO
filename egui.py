#!/usr/bin/env python2

"""
@file       egui.py
@author     Team 3 (Muhammad Fazrey Bin Zainal, Kaushik Thirumavalavan, Akmal Rusyaidi Bin Muhammad Tajwid,
            Muhammad Zulhilman Bin Mohd Shaini, Chang Keng Jethro, Janaishwaran)
@course     Robotics Systems Engineering, Singapore Institute of Technology
@module     SEP
@date       24/07/2023
@brief      Graphic User Interface (GUI) using Tkinter for ease of selection for the desired locations throughout the arena.
"""


import Tkinter as tk
import numpy as np
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8

# Function that gets executed when a button is clicked
def button_clicked(num):

    if num == 10:
        # If the button clicked is number 10, publish the number to '/stopButton' topic
        stop_pub.publish(num)
    else:
        # If any other button is clicked, publish the number to '/button' topic
        pub.publish(num)

# Create the main window
window = tk.Tk()
window.title("Tourist Attractions")

# IMPORTANT DEFS
# Initialize ROS node and set it to be anonymous, declare the publishers pub and stop_pub
pub = rospy.Publisher('/button', Int8, queue_size=1)  # Publisher for '/button' topic
stop_pub = rospy.Publisher('/stopButton', Int8, queue_size=1)  # Publisher for '/stopButton' topic

rospy.init_node('simple_gui', anonymous=True)

# Calculate the size in pixels for a 20 by 20 cm window
width_pixels = window.winfo_fpixels("10c")
height_pixels = window.winfo_fpixels("10c")
window.minsize(int(width_pixels), int(height_pixels))

# Create a header label
header_label = tk.Label(window, text="Team 3 Limo Navigation", font=("Helvetica", 28, "bold"), fg="#ffffff", bg="#333333")
header_label.pack(pady=20)

# Create a frame to hold the buttons
button_frame = tk.Frame(window, bg="#f2f2f2")
button_frame.pack()

# Configuration variables
ht = 4  # Height of the buttons
wt = 10  # Width of the buttons
ft = ("Helvetica", 10)  # Font style and size for the buttons
bk = "#0099cc"  # Background color of the buttons
fg = "white"  # Foreground (text) color of the buttons


# Create buttons with custom style
button_origin = tk.Button(
    button_frame,
    text="Origin",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(0),
    bg="#0099cc",
    fg="white"
)

button_sentosa = tk.Button(
    button_frame,
    text="Sentosa",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(1),
    bg="#0099cc",
    fg="white"
)

button_wingsoftime = tk.Button(
    button_frame,
    text="WOT",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(2),
    bg="#0099cc",
    fg="white"
)

button_uss = tk.Button(
    button_frame,
    text="USS",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(3),
    bg="#0099cc",
    fg="white"
)

button_seaaquarium = tk.Button(
    button_frame,
    text="SEA",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(4),
    bg="#0099cc",
    fg="white"
)

button_fortsiloso = tk.Button(
    button_frame,
    text="Siloso",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(5),
    bg="#0099cc",
    fg="white"
)

button_merlion = tk.Button(
    button_frame,
    text="Merlion",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(6),
    bg="#0099cc",
    fg="white"
)

button_rainbow = tk.Button(
    button_frame,
    text="Reef",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(7),
    bg="#0099cc",
    fg="white"
)

button_ifly = tk.Button(
    button_frame,
    text="iFly",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(8),
    bg="#0099cc",
    fg="white"
)

button_stop = tk.Button(
    button_frame,
    text="STOP",
    font=ft,
    width=wt,
    height=ht,
    command=lambda: button_clicked(10),
    bg="#cc0000",
    fg="white"
)



# Pack buttons in a grid layout
button_origin.grid(row=1, column=1, padx=10, pady=10)
button_sentosa.grid(row=2, column=1, padx=10, pady=10)
button_uss.grid(row=1, column=2, padx=10, pady=10)
button_stop.grid(row=1, column=3, padx=10, pady=10)
button_rainbow.grid(row=1, column=0, padx=10, pady=10)
button_ifly.grid(row=0, column=0, padx=10, pady=10)
button_wingsoftime.grid(row=2, column=2, padx=10, pady=10)
button_seaaquarium.grid(row=2, column=0, padx=10, pady=10)
button_fortsiloso.grid(row=0, column=2, padx=10, pady=10)
button_merlion.grid(row=0, column=1, padx=10, pady=10)

# Set the background color of the main window to light gray (#f2f2f2)
window.configure(bg="#f2f2f2")

# This is the main loop of the GUI, which keeps the window running and responsive to user actions
window.mainloop()

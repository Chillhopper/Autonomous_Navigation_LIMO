# Autonomous_Navigation_LIMO
## Introduction
This repository contains the LIMO navigation files for autonomous navigation. It uses the ROS Navigation stack to navigate autonomously around vehicles using the ```dijkstra``` algorithm for mapping the area to perform navigation around obstacles.

### Prerequisites
- Python 2.X
- ROS
- TurtleBOT3 Gazebo

## Usage Instructions

To run the `arl.sh` script, follow these steps:

1. **Run `arl.sh` Script (RECOMMENDED)**:
   - Navigate to the directory containing `arl.sh`.
   - Execute the script using the command:
     ```bash
     ./arl.sh
     ```
   **OTHERWISE** you may choose to execute each command manually which will sequentially start all the necessary Gazebo and program setup commands.

2. **Manual Command Execution**:
   Alternatively, you can manually execute the following commands:
   - Start LIMO:
     ```bash
     roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
     ```
   - Launch Navigation:
     ```bash
     roslaunch limo_bringup limo_navigation_diff.launch
     ```
   - Run `emain.py`:
     ```bash
     rosrun arena emain.py
     ```
   - Execute `egui.py`:
     ```bash
     rosrun arena egui.py
     ```
   - Start `poseHub.py`:
     ```bash
     rosrun poseHub.py
     ```

Choose the method that best fits your workflow. The script provides a convenient one-step option, while manual execution offers more control over each process.


## Contact
For any queries, you can reach out to ```kaushikt2000@gmail.com``` .


![LimoNav](https://github.com/Chillhopper/LIMO_NAV_Archive/assets/68851163/1484c07d-5e5a-4be7-923a-6b488c7bb5df)


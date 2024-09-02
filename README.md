# Autonomous-Mobile-Robot-Navigation

This repository contains the submission for ENPM809Y Final Project - Navigate autonomously using SLAM and action-client nodes in ROS2. The waypoints are determined by reading the positions of batteries placed in the environment through cameras. The TF tree is consulted to obtain the current position of the robot and the waypoint locations.

# Maze-Navigation-and-Object-Detection

This repository contains the submission for ENPM809Y RWA 3  - A ROS package to move a turtlebot in a maze. To move through the maze, the turtlebot relies on Aruco markers. While the turtlebot navigates the maze, it will need to ﬁnd and report objects found in the environment. The order in which the waypoint should be navigated is got by reading a aruco marker. 

![Video GIF](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation/blob/main/final_demo.gif)

## Repository Structure
- **group5_final**: Package for autonomous waypoint navigation.
- **mage_msgs**: Package for defining custom ros messages
- **ros2_aruco**: Package for reading aruco markers
- **turtlebot3_navigation2**: Package containing turtlebot3 navigation
- **final_project**: Package for SLAM and simulating turtlebot3 on gazebo

## Prerequisites
- Before running the code, ensure that you have the following installed:
  - ROS2 (recommended version: Humble)
  - Gazebo
  - OpenCV
  - OpenCV Contrib
  - numpy (version < 2)

## Setup Instructions

1. **Run the following lines of code to set up the environment**
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip3 uninstall opencv-python opencv-contrib-python
   pip3 install opencv-python opencv-contrib-python
   pip3 install "numpy<2"
   echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
   source ~/.bashrc
   ```
2. **Create a Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
3. **Copy Packages**:
   - Paste the packages, which are inside the zip folder `suhas99_enpm809y_final.zip`, into the src folder of your workspace. Also paste the package `group5_final` in the same src folder.

4. **Download Models**:
   - Download the models required for simulation from [HERE](https://drive.google.com/drive/folders/1MwollaZJ7j2-e4DEU-VY649KMrPtxQOj?usp=sharing).
   - Paste these models inside the `final_project` package.
     
4. **Build and Source the Packages**:
   ```bash
   cd ~/ros2_ws
   colcon build --allow-overriding mage_msgs ros2_aruco_interfaces turtlebot3_navigation2
   source install/setup.bash
   ```
   
## Running the Simulation

1. **Launch Turtlebot3 Waffle in Gazebo (Custom World) and on RVIZ (with map)**:
   ```bash
   ros2 launch final_project final_project.launch.py use_sim_time:=True
   ```
   
2. **Set the initial pose of the robot by clicking on `2D Pose Estimate` on RVIZ**:

![alt text](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation/blob/main/initial_pose.png?raw=false)
   
3. **Launch the file `tbot_nodes.launch.py` to start the battery broadcasting nodes**:
   ```bash
   ros2 launch group5_final tbot_nodes.launch.py use_sim_time:=True
   ```
   
3. **Launch the file `tbot_pub.launch.py` (node) to get the waypoints/goals**:
   ```bash
   ros2 launch group5 tbot_pub.launch.py
   ```

4. **Run the node `tbot_through_poses` to call action `NavigateThroughPoses` OR Run the node `tbot_follow_waypoints` to call action `FollowWaypoints`for navigation**:
   ```bash
   ros2 run group5_final tbot_through_poses 
   ```
    OR
   ```bash
   ros2 run group5_final tbot_follow_waypoints
   ```
   
4. **Results**:
   - Wait till the robot fully navigates and stops
   - The order in which the robot mavigates the waypoints is set in the params file and by reading the aruco marker
   - For more information, refer to the project description: [ENPM809Y_FINAL](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation/blob/main/FINAL_ENPM809Y_FALL2023-v1.0.pdf).

## Acknowledgments

This project utilizes code and resources from the [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) repository. We would like to thank the TurtleBot3 team for their contributions and the open-source community for providing these valuable resources.

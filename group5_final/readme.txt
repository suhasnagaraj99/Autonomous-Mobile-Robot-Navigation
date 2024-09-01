ENPM 809Y : Final Project Submission Group 5

This package contains the code for Final Project (Waypoint following robot)

Before running the file make sure all the dependencies are installed as mentioned in the Final Project Statement pdf.


Follow these steps to run the package 
1. Download the package and unzip the contents into the src folder of a workspace which would also contain the following pakages : mage_msgs, final_project, ros2_aruco(will contain ros2_aruco and ros2_aruco_interfaces) (All these have been provided by the professor).

2. Colcon build the packages from the root folder of the workspace and make sure the turtle bot model is set to waffle.

3. Launch the turtlebot in gazebo and rviz with the map using the following command : ros2 launch final_project final_project.launch.py
(Wait till gazebo and rviz opens and wait for 5 more seconds to ensure the launched nodes are started cleanly )

4. Launch the file tbot_nodes.launch.py to start the following nodes : camera1_broadcaster, camera2_broadcaster, camera3_broadcaster, camera4_broadcaster and camera5_broadcaster using the following command: ros2 launch group5_final tbot_nodes.launch.py
(Wait for 5-10 seconds to ensure all the nodes are started cleanly) 

5. To extract the waypoints, launch the tbot_pub launch file by using the following command : ros2 launch group5_final tbot_pub.launch.py
(Wait for 5 seconds to ensure that the node is started cleanly)

6. After all the previous nodes are started, run the node (client) to call the action to execute the waypoint navigation using following command: ros2 run group5_final tbot_follow_waypoints
(The turtlebot takes 5 seconds to initialize and 10 seconds to start waypoint navigation)

7. The turtle bot moves through the waypoints and navigates through the maze


Note: 
1. Source the workspace before running any executable from the package
2. If any node related warnoings appear on the terminal screen, please ignore them / wait for few seconds.
3. If any node related errors appear on the terminal, kindly relaunch the nodes again (follow steps 3 to 6 again)
4. We are using follow waypoints action instead of navigate through poses action as it gave better results. But we have included the node(client) which calles navigate through poses action. Please run the executable tbot_through_poses to launch this node.
5. After reaching the waypoint, the turtlebot might take some time to orient itself before moving towards the next waypoint, kindly wait for the exectution to complete.
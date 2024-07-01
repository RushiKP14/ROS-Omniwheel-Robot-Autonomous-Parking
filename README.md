# Omniwheel Mobile Robot Simulation for Autonomous Parking

## Overview

This project simulates an omniwheel mobile robot performing autonomous parking in a self-defined complex environment using Gazebo. The repository contains all necessary files, including the URDF for the robot, custom-made worlds, configuration files for the ROS navigation stack, scripts for sending goals, and launch files to start the Gazebo world and necessary nodes.

### Method

1. **URDF Creation:** 
   - Created a URDF file using the SolidWorks to URDF conversion plugin.
   - Added a LiDAR sensor and control plugins to the URDF.

   ![Gazebo](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/59167cee-e64d-424b-8e2a-c0ce48c9a6f1)

3. **Autonomous Parking:**
   - Utilized the ROS navigation stack.
   - Created a map of the world using `slam_gmapping`.
   - Configured parameters for the `map_server`, `amcl`, and `move_base` nodes to enable autonomous parking.

## Installation

Follow these steps to set up the project on your PC:

1. **Clone the Repository:**
	Clone the repository inside the **/src** folder of your workspace. (e.g. ~/catkin_ws/src)
   ```bash
   git clone https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking.git
   ```
2. **Build the Workspace:**
   Navigate to your catkin workspace and build the project.
   ```bash
   cd ~/catkin_ws
   catkin_make --only-pkg-with-deps omnirobot
   source devel/setup.bash
   ```
3. **Run the Simulation:**
	Launch the Gazebo world and necessary nodes.
	```bash
	roslaunch omniwheel task.launch
	```
	![launch](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/acc4bc9b-15a9-41e6-9e33-73fd61d26730)
4. **Send Navigation Goal via Terminal:**
	Use the following command to send a goal to the `move_base` ROS action server.
	```bash
	rosrun omniwheel send_goal <x y w>
	```
	- Ensure that the coordinates you provide do not correspond to locations with obstacles on the map.

	![send_goal](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/08d347fb-2aa0-4975-adba-777420d43008)

5. **Send Navigation Goal via RViz:**
	Alternatively, you can send the final goal using RViz.
	```bash
	roslaunch omniwheel display.launch
	```
	- An RViz window will open.

	![RViz](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/b5e418a9-438d-4198-9d2e-1dc18e8abcf4)

	- Perform **Initial Pose Estimation** using the `2D Pose Estimate` button in the RViz menu.

	![Screenshot 2024-07-01 103830](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/a00ab99a-d732-4308-9389-6ec73db3b2d9)

	- Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
 	- Launch keyboard teleoperation node to precisely locate the robot on the map.
	```bash
 		rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
	- Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the robot on the map which is displayed with tiny red arrows.

	![localisation-gif](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/5be56b12-f398-44fe-85ba-5aa128aec5eb)

	- Terminate the keyboard teleoperation node by entering `Ctrl` + `C` to the teleop node terminal in order to prevent different cmd_vel values are published from multiple nodes during Navigation.

	- **Set the navigation goal** using the `2D Nav Goal` button in the RViz menu.

	![Screenshot 2024-07-01 103850](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/496d30e6-8dde-44c0-b0d3-48446c595ddf)

	- Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
	- As soon as x, y, θ are set, robot will start moving to the destination immediately.

	![Rviz_nav](https://github.com/RushiKP14/ROS-Omniwheel-Robot-Autonomous-Parking/assets/156124606/6083cb28-7794-4ea6-9477-9a80f7c0b4f2)

## Features

- **Omniwheel Robot Simulation:** Realistic simulation using a URDF model with integrated LiDAR sensor.
- **Autonomous Navigation:** Utilizes ROS navigation stack for autonomous parking.
- **Custom Environments:** Includes custom-made Gazebo worlds for varied and complex scenarios.

## Contact

For any questions or feedback, please contact:

- **Name:** Rushi Padhiyar
- **Email:** rushikp14gmail.com

## Dependencies

- ROS (Robot Operating System)
- Gazebo
- RViz
- `slam_gmapping` package
- `move_base` package
- `amcl` package

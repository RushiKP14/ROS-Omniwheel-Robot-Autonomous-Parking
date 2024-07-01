# Omniwheel Mobile Robot Simulation for Autonomous Parking

## Overview

This project simulates an omniwheel mobile robot performing autonomous parking in a self-defined complex environment using Gazebo. The repository contains all necessary files, including the URDF for the robot, custom-made worlds, configuration files for the ROS navigation stack, scripts for sending goals, and launch files to start the Gazebo world and necessary nodes.

### Method

1. **URDF Creation:** 
   - Created a URDF file using the SolidWorks to URDF conversion plugin.
   - Added a LiDAR sensor and control plugins to the URDF.
2. **Autonomous Parking:**
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
   Navigate to your catkin workspace and build the project:
   ```bash
   cd ~/catkin_ws
   catkin_make --only-pkg-with-deps omnirobot
   source devel/setup.bash
   ```
3. **Run the Simulation:**
	Launch the Gazebo world and necessary nodes:
	```bash
	roslaunch omniwheel task.launch
	```
4. **Send Navigation Goal via Terminal:**
	Use the following command to send a goal to the `move_base` ROS action server:
	```bash
	rosrun omniwheel send_goal <x y w>
	```
	- Ensure that the coordinates you provide do not correspond to locations with obstacles on the map.

5. **Send Navigation Goal via RViz:**
	Alternatively, you can send the final goal using RViz:
	```bash
	roslaunch omniwheel display.launch
	```
	- An RViz window will open.
	- Perform Initial Pose Estimation using the `2D Pose Estimate` button in the RViz menu.
	- Set the navigation goal using the `2D Nav Goal` button in the RViz menu.

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

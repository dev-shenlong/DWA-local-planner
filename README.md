![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
# DWA Local Planner
This is an implementation of a DWA local planner for ROS2. It has been tested on the **turtlebot3**. 

## Setup Instructions
This assumes that **gazebo** and **ROS2 Humle** have been setup.

### Install Dependencies
Install Nav2 and cartographer to be used along with the turtlebot.
```bash
  sudo apt install ros-humble-cartographer 
  sudo apt install ros-humble-cartographer-ros
  sudo apt install ros-humble-navigation2 
  sudo apt install ros-humble-nav2-bringup
```

### Installing the simulation repositories
In the source folder of your workspace.
```bash
mkdir _ws
cd _ws
mkdir src
cd src
```
Clone the following repositories.
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel
```
Once cloned build them
```bash
  cd ..
  colcon build
  source install/setup.bash
```
### Installing the planner
Run the following commands in the src directory of your ws
```bash
  git clone https://github.com/dev-shenlong/DWA-local-planner.git
  cd ..
  colcon build
  source install/setup.bash
```
## Running the simulation
You can launch the simulation using the launch file present in the package
```bash
  ros2 launch dwa_custom_planner sim.launch.py 
```
The planner can be run using the following command
```bash
  ros2 run dwa_custom_planner dwa_custom_planner
```

## Assigning a goal
1. Open the RViz window started from the launch file
2. Use "2D Goal Pose" tool
3. Click anywhere in the map to set the goal


  

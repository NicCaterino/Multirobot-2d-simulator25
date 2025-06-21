# Multirobot Simulator with ROS Support

This project extends a 2D robot simulator to support ROS communication. The simulator allows you to control multiple robots on a map using standard ROS topics and visualize sensor data in real-time.

## Overview

The simulator loads a PNG map file and spawns robots that can be controlled remotely through ROS topics. Each robot publishes its odometry data and can be equipped with laser sensors that publish scan data. The system supports multiple robots with separate namespaces to avoid topic conflicts.

## Quick Start

After cloning this repository, you can get the simulator running in just a few steps. The project includes a simple script that handles compilation and execution.

```bash
git clone https://github.com/your-username/multirobot-ros-simulator.git
cd multirobot-ros-simulator
./run.sh
```

The script will prompt you with three options. Choose option 1 to build only, option 2 to run only (if already built), or option 3 to build and run together. Most users will want option 3 for the first time setup.

After building, the script asks which configuration to use. Option 1 loads a single robot with laser sensor, while option 2 loads two robots where only the first has a laser sensor.

## Robot Control

Once the simulator is running, you can control the robots using standard ROS commands. Open a new terminal and use rostopic pub to send velocity commands.

```bash
# Move robot forward
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -r 10

# Rotate robot
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}' -r 10

# Stop robot
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
```

For the second robot in multi-robot configuration, use `/robot_1/cmd_vel` instead.

## What You'll See

The simulator opens two windows when running. The bigger window shows the actual simulation with the map in black and white, robots as black circles, and laser rays as gray lines. You can press 'q' or ESC in this window to exit the simulator.
The smaller one is RVIZ in order to check the simulation params.


## Manual Build (Alternative)

If you prefer to build manually instead of using the script, you can do so with standard catkin commands:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch multirobot_simulator simulator.launch
```

## Project Structure

The repository contains a catkin workspace in the `catkin_ws` directory with the main package in `src/exercise`. Map files are located in the `map` directory, and configuration files are in `test_data`. The `run.sh` script handles the build process and configuration selection automatically.

## ROS Topics

The system provides standard ROS interfaces. Control topics are `/robot_N/cmd_vel` for velocity commands. Sensor topics include `/robot_N/odom` for odometry data and `/robot_N/base_scan` for laser scans. All spatial relationships are maintained through TF transforms published automatically by the simulator.

## Customization

You can modify the simulation by editing JSON configuration files in the `test_data` directory. These files define robot positions, sensor parameters, and map files. The system supports adding more robots or changing sensor configurations as needed for your specific use case.

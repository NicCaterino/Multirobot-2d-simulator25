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
## Code Structure

The project follows a wrapper pattern design that extends the original simulator classes with ROS functionality without modifying the base code.

### Core Architecture

**Base Classes (Original Simulator):**
- `World`: Manages the map, collision detection, and object rendering
- `Robot`: Handles robot movement, physics, and basic drawing
- `Lidar`: Simulates laser scanner with raycast algorithms
- `WorldItem`: Base class for all objects in the simulation world

**ROS Wrapper Classes:**
- `RobotROS`: Extends Robot with ROS topic publishing/subscribing
- `LidarROS`: Extends Lidar with sensor_msgs/LaserScan publishing
- `ConfigParser`: Handles JSON configuration file parsing

**Integration Layer:**
- `MultiRobotSimulator`: Main ROS node that orchestrates everything
- Launch files and configuration management

### Key Design Decisions

The wrapper pattern was chosen to preserve the original simulator functionality while adding ROS capabilities. Each RobotROS instance subscribes to cmd_vel topics for control and publishes odometry data. LidarROS instances convert the ray-casting results into standard ROS LaserScan messages.

The configuration system uses JSON files to define robot positions, sensor parameters, and parent-child relationships. This allows flexible multi-robot setups without code changes.

### File Organization

**Source Files (`src/`):**
- `main.cpp` - Main ROS node
- `robot.cpp` - Base robot class
- `lidar.cpp` - Base lidar class  
- `world.cpp` - Simulation world
- `robot_ros.cpp` - ROS robot wrapper
- `lidar_ros.cpp` - ROS lidar wrapper
- `config_parser.cpp` - JSON configuration parser

**Header Files (`include/multirobot_simulator/`):**
- `robot.h`, `lidar.h`, `world.h`, `types.h` - Base classes
- `robot_ros.h`, `lidar_ros.h`, `config_parser.h` - ROS wrappers

**Configuration (`config/`):**
- `rviz_config_laser.rviz` - RVIZ visualization setup with laser and odometry visualization
- Launch files for different robot configurations

**Test Data (`test_data/`):**
- `multirobot.json` - Multi-robot configuration
- Map PNG files for simulation environments


### ROS Topics
The system provides standard ROS interfaces. Control topics are /robot_N/cmd_vel for velocity commands. Sensor topics include /robot_N/odom for odometry data and /robot_N/base_scan for laser scans. All spatial relationships are maintained through TF transforms published automatically by the simulator.
### Customization
You can modify the simulation by editing JSON configuration files in the test_data directory. These files define robot positions, sensor parameters, and map files. The system supports adding more robots or changing sensor configurations as needed for your specific use case.


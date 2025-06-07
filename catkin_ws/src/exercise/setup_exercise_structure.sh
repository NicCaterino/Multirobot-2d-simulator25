#!/bin/bash

# Script per completare la struttura nella cartella exercise esistente

echo "=== Setup struttura progetto multirobot simulator ==="

# Assumendo di essere nella cartella exercise/
# Crea le cartelle mancanti
mkdir -p launch config

# Crea la sottocartella per gli header
mkdir -p include/multirobot_simulator

# Crea .gitignore ottimizzato per ROS
cat > .gitignore << 'EOF'
# Build files
build/
devel/
install/
bin/
*.o
*.so

# IDE and editor files
.vscode/
.idea/
*~
*.swp
*.swo

# Python
*.pyc
__pycache__/

# OS
.DS_Store
Thumbs.db
EOF

# Aggiorna il package.xml per ROS
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>multirobot_simulator</name>
  <version>1.0.0</version>
  <description>Multi-Robot 2D Simulator with ROS support</description>

  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  
  <!-- ROS core dependencies -->
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>
  
  <!-- JSON parsing -->
  <build_depend>jsoncpp</build_depend>
  
  <!-- Runtime dependencies -->
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>jsoncpp</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
EOF

# Crea CMakeLists.txt completo
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(multirobot_simulator)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

## Find jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros
  DEPENDS JSONCPP
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

## Declare a C++ library (se hai classi da condividere)
add_library(${PROJECT_NAME}
  src/world.cpp
  src/robot.cpp
  src/lidar.cpp
  # Aggiungi altri file .cpp se necessari
)

## Add dependencies
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/main.cpp)

## Add dependencies
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

## Install targets
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
EOF

# Crea il file launch
cat > launch/simulator.launch << 'EOF'
<launch>
  <!-- Arguments -->
  <arg name="config_file" default="$(find multirobot_simulator)/config/cappero_1r.json"/>
  <arg name="use_rviz" default="true"/>
  <arg name="rviz_config" default="$(find multirobot_simulator)/config/rviz_config.rviz"/>
  
  <!-- Static transform publisher for map frame -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_odom" 
        args="0 0 0 0 0 0 map odom"/>
  
  <!-- Multi-robot simulator node -->
  <node name="multirobot_simulator_node" pkg="multirobot_simulator" type="multirobot_simulator_node" 
        args="$(arg config_file)" output="screen">
    <param name="config_file" value="$(arg config_file)"/>
  </node>
  
  <!-- RViz visualization -->
  <node if="$(arg use_rviz)" name="rviz" pkg="rviz" type="rviz" 
        args="-d $(arg rviz_config)" required="false"/>
        
</launch>
EOF

# Crea configurazione RVIZ di base
cat > config/rviz_config.rviz << 'EOF'
Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Time
    Name: Time

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: map
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Fixed Frame: map
  Name: root
  Tools:
    - Class: rviz/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 20
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785398
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: ~
EOF

# Crea un file JSON di esempio per il test
cat > config/cappero_1r.json << 'EOF'
{
    "map": "cappero_laser_odom_diag_2020-05-06-16-26-03.png",
    "items": [
        {
            "id": 0,
            "type": "robot",
            "frame_id": "robot_0_base_link",
            "namespace": "robot_0",
            "radius": 0.5,
            "max_rv": 0.1,
            "max_tv": 0.5,
            "pose": [20, 8, 0],
            "parent": -1
        },
        {
            "id": 1,
            "type": "lidar",
            "frame_id": "robot_0_laser",
            "namespace": "robot_0",
            "fov": 6.28,
            "max_range": 10.0,
            "num_beams": 360,
            "pose": [0, 0, 0],
            "parent": 0
        }
    ]
}
EOF

echo ""
echo "=== Struttura completata! ==="
echo ""
echo "Struttura finale:"
echo "exercise/"
echo "├── CMakeLists.txt          ✓ Completato per ROS"
echo "├── package.xml             ✓ Aggiornato per ROS"
echo "├── src/                    ✓ (i tuoi file esistenti)"
echo "├── include/                ✓ (+ sottocartella multirobot_simulator/)"
echo "├── bin/                    ✓ (i tuoi file esistenti)"
echo "├── launch/                 ✓ Creato"
echo "│   └── simulator.launch    ✓ Creato"
echo "└── config/                 ✓ Creato"
echo "    ├── rviz_config.rviz    ✓ Creato"
echo "    └── cappero_1r.json     ✓ Esempio creato"
echo ""
echo "Prossimi passi:"
echo "1. Installa dipendenze: sudo apt install libjsoncpp-dev"
echo "2. Sposta i file .h in include/multirobot_simulator/"
echo "3. Adatta il codice per ROS"
echo "4. Compila con catkin_make"
echo "5. Test con: roslaunch multirobot_simulator simulator.launch"

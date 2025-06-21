

#!/bin/bash

# Simple launcher for multirobot simulator

echo "Multirobot Simulator"
echo "1) Build"
echo "2) Run" 
echo "3) Build and Run"
read -p "Choose : " choice
choice=${choice:-3}

case $choice in
    1)
        echo "Building..."
        cd catkin_ws
        catkin_make
        source devel/setup.bash
        echo "Build complete"
        ;;
    2)
        echo "Running..."
        echo "Config: 1) Single robot  2) Multi robot"
        read -p "Choose config [1]: " config
        config=${config:-1}
        cd catkin_ws
        source devel/setup.bash
        if [ "$config" = "2" ]; then
            roslaunch multirobot_simulator simulator.launch config_file:=/home/lattinone/Desktop/Multirobot-2d-simulator25/test_data/multirobot.json
        else
            roslaunch multirobot_simulator simulator.launch
        fi
        ;;
    3)
        echo "Building..."
        cd catkin_ws
        catkin_make
        source devel/setup.bash
        echo "Build complete. Starting simulator..."
        echo "Config: 1) Single robot  2) Multi robot"
        read -p "Choose config [1]: " config
        config=${config:-1}
        if [ "$config" = "2" ]; then
            roslaunch multirobot_simulator simulator.launch config_file:=/home/lattinone/Desktop/Multirobot-2d-simulator25/test_data/multirobot.json
            roslaunch multirobot_simulator simulator.launch
        fi
        ;;
    *)
        echo "Invalid choice"
        ;;
esac

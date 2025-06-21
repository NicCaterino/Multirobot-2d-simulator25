#!/bin/bash

echo "Multirobot Simulator"
echo "1) Build"
echo "2) Run" 
echo "3) Build and Run"
read -p "Choose [3]: " choice
choice=${choice:-3}

run_simulator() {
    echo "Config: 1) Single robot  2) Multi robot"
    read -p "Choose config [1]: " config
    config=${config:-1}
    cd catkin_ws
    source devel/setup.bash
    if [ "$config" = "2" ]; then
        roslaunch multirobot_simulator simulator.launch config_file:=../test_data/multirobot_test.json
    else
        roslaunch multirobot_simulator simulator.launch
    fi
}

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
        run_simulator
        ;;
    3)
        echo "Building..."
        cd catkin_ws
        catkin_make
        if [ $? -eq 0 ]; then
            source devel/setup.bash
            echo "Build complete. Starting simulator..."
            cd ..
            run_simulator
        else
            echo "Build failed!"
            exit 1
        fi
        ;;
    *)
        echo "Invalid choice"
        ;;
esac
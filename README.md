# Multirobot Simulator - ROS

Simulatore 2D con robot che si muovono su mappe. Ho esteso il simulatore base aggiungendo ROS per controllare i robot da remoto.

## Come installare

Serve Ubuntu 20.04 con ROS Noetic. 

Per compilare:
```bash
cd /path/to/catkin_ws
catkin_make
source devel/setup.bash
```

## Come usare

Lancia il simulatore:
```bash
roslaunch multirobot_simulator simulator.launch
```

Si aprono due finestre: una con la mappa (OpenCV) e RVIZ.

## Muovere il robot

In un nuovo terminale:
```bash
# Vai avanti
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -r 10

# Gira
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}' -r 10

# Stop
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
```

Il robot si dovrebbe muovere nella finestra OpenCV.

## Cosa vedere in RVIZ

Aggiungi questi elementi:
- **TF** - vedi i frame del robot
- **Odometry** su topic `/robot_0/odom` - vedi dove va il robot  
- **LaserScan** su topic `/robot_0/base_scan` - vedi cosa "vede" il laser

Imposta Fixed Frame su `map`.

## Topics disponibili

- `/robot_0/cmd_vel` - manda comandi al robot
- `/robot_0/odom` - posizione del robot
- `/robot_0/base_scan` - dati del laser

## File di configurazione

Il file JSON in `config/cappero_1r.json` definisce robot e sensori.



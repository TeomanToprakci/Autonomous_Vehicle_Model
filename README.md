# Autonomous_Vehicle_Model

This project demonstrates the design and implementation of a Lidar-based autonomous robot system for indoor environments such as warehouses or production facilities. The robot can localize itself, detect obstacles, and navigate to user-defined target points without human intervention.


## 📌 Features

- SLAM-based map creation using YDLidar X4 Pro
- Obstacle detection and avoidance
- AMCL-based localization
- Goal-directed navigation using ROS 2 Nav2 stack
- Button-based target selection interface via Arduino
- Custom `cmd_vel` → PWM motor control over serial
- Real-world testbed in a 14m² demo warehouse environment

---

## 🧰 Technologies Used

| Technology       | Purpose                               |
|------------------|----------------------------------------|
| ROS 2 (Humble)   | Core robot framework                   |
| RViz 2           | Visualization of robot state and map   |
| YDLidar X4 Pro   | Lidar sensor for mapping and perception|
| Arduino Leonardo | Button input + motor PWM interface     |
| Ubuntu 22.04     | Operating system for ROS 2 environment |
| L298N Driver     | DC motor driver used in real robot     |

---

## 📁 Project Structure
```
ros2_ws/
├── src/
│ └── robot_control/
│ │ ├── launch/
│ │ │ └── robot_nodes_launch.py
│ │ │ └── rviz_launch.py
│ │ ├── config/
│ │ │ └── nav2_params.yaml
│ │ ├── urdf/
│ │ │ └── robot.urdf
│ │ └── Nodes/
│ │ │ └── motor_controller_serial.py
│ │ │ └── tf_publisher.py
│ │ │ └── button_navigation_node.py
│ │ ├── rviz/
│ │ │ └── nav2_custom_view.rviz
~/miniwarehouse.yaml (Map File)
~/miniwarehouse.pgm
```
---

## Requirements and Launch Order
This project requires Nav2 packages and YDLiDAR drivers to launch. Here is the launch order and terminal commands to start properly:

1- ros2 launch ydlidar_ros2_driver ydlidar_launch.py scan_topic:=scan_raw

2- ros2 launch robot_control robot_nodes_launch.py

3- ros2 launch nav2_bringup localization_launch.py map:=/home/teomant/miniwarehouse.yaml use_sim_time:=false

4- ros2 launch nav2_bringup navigation_launch.py params_file:=/home/teomant/ros2_ws/src/robot_control/config/nav2_params.yaml use_sim_time:=false

5- ros2 launch robot_control rviz_launch.py <----(if observation is needed)

<p float="left"> <img src="/rviz_screenshot.png" width="400" </p>




## 🧠 Notes and Limitations
Localization drift: Due to friction-based wheel slippage, AMCL sometimes fails to match scan data with the map.

Odometry is based on dead reckoning (PWM estimation), not encoders.

Real tests were performed in a 14m² warehouse demo area.



## 📄 License
This project is licensed under the MIT License.

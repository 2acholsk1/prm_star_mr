# PRM* PROJECT

## Installation Guide

### Linux Ubuntu

1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/)

2. Install [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extenstion in VS Code. Do not check *Execute In WSL* in DEV Container: settings

3. Check if docker group exists, if not: 

    3.1 Create the docker group
        `sudo groupadd docker`

    3.2 Add user to the docker group
        `sudo usermod -aG docker $USER`

    3.3 Log out and log back in so that your group membership is re-evaluated.

4. Use View->Command Palette... or Ctrl+Shift+P to open the command palette. Search for the command Dev Containers: Reopen in Container and execute it. This will build docker container for your. It will take a while - sit back or go for a coffee.

## SETUP

1. After docker setup, run bash script from folder utils:
    `bash /utils/env_setup.sh`
    
    It will install required dependencies and setup .bashrc, then if you open new terminal you have not to source to ROS2
    1.1 If you work in the same terminal, source ros2
        `source /opt/ros/humble/setup.bash`

2. Build tha ROS2 package in /home/ws with command:
    `colcon build --symlink-install`

ERRORS:
    If you have problem with display windows, use:
    `EXPORT DISPLAY=:0`

## EXAMPLE OF NAV2

1. After SETUP, set key environment variables:

    `source install/setup.bash`
    
    `export TURTLEBOT3_MODEL=waffle`

    `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models`

2. In the same terminal, run:

    ### Default config:

    `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`

    ### Our config:

    Default launch file have to be changed, because it's poorly done. It is located in:
    ```bash
    /opt/ros/humble/share/nav2_bringup/launch/tb3_simulation_launch.py
    ```
    Fixed file is provided in: 
    ```bash
    /home/ws/src/utils/tb3_simulation_launch.py
    ```
    
    Launch using:
    ```bash
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=/home/ws/src/prm_star/config/nav2_params.yaml robot_sdf:=/home/ws/src/ackermann_v2/src/description/ackermann_vehicle.urdf rviz_config_file:=/home/ws/src/ackermann_v2/rviz/rviz_config.rviz
    ```


## Current state:

- we have working car model with Ackermann kinematics,

- it can be moved with nav2,

- currently it's working with "nav2_navfn_planner/NavfnPlanner", I tried changing it to our planner which is currently straightline planner but it didn't move, don't know why xD

- we need to implement proper robot_model_type as planner currently thinks it has a turtlebot kinematics:
![alt text](img/image-1.png)
I think that what we need to do

### Config files:

ackermann_v2/config/nav2_params2.yaml - nav2 parameters

ackermann_v2/src/decription/ackermann_vehicle.urdf - our vehicle model

ackermann_v2/rviz/urdf_config.rviz - Rviz config file

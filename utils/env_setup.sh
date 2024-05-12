#!/bin/bash

sudo apt-get update

sudo apt-get install ros-humble-rviz2

line_to_add="source /opt/ros/humble/setup.bash"

if ! grep -qF "$line_to_add" ~/.bashrc; then
    echo "$line_to_add" >> ~/.bashrc
    echo "Line added to .bashrc."
else
    echo "Line already exist in .bashrc."
fi

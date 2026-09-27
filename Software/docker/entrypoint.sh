#!/bin/bash
set -e

# Carga el entorno base de ROS Melodic
source /opt/ros/melodic/setup.bash

# Si ya existe un workspace compilado, lo carga también
if [ -f /home/ros/catkin_ws/devel/setup.bash ]; then
    source /home/ros/catkin_ws/devel/setup.bash
fi

exec "$@"

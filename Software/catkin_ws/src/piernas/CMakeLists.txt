cmake_minimum_required(VERSION 3.0.2)
project(piernas)

## Dependencias de catkin necesarias
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## Declaración del mensaje personalizado Alba.msg
add_message_files(
  FILES
  Alba.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

## ==========================================================
## INSTALACIÓN DE SCRIPTS DE PYTHON
## ==========================================================
catkin_install_python(PROGRAMS
  scripts/state_machine.py
  scripts/movimiento.py
  scripts/Leg_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
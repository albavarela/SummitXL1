# CMake generated Testfile for 
# Source directory: /home/ros/catkin_ws/src/people/leg_detector
# Build directory: /home/ros/catkin_ws/build/leg_detector
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_leg_detector_roslaunch-check_launch "/home/ros/catkin_ws/build/leg_detector/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/ros/catkin_ws/build/leg_detector/test_results/leg_detector/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/ros/catkin_ws/build/leg_detector/test_results/leg_detector" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/ros/catkin_ws/build/leg_detector/test_results/leg_detector/roslaunch-check_launch.xml\" \"/home/ros/catkin_ws/src/people/leg_detector/launch\" ")
add_test(_ctest_leg_detector_roslint_package "/home/ros/catkin_ws/build/leg_detector/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/ros/catkin_ws/build/leg_detector/test_results/leg_detector/roslint-leg_detector.xml" "--working-dir" "/home/ros/catkin_ws/build/leg_detector" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/ros/catkin_ws/build/leg_detector/test_results/leg_detector/roslint-leg_detector.xml make roslint_leg_detector")
subdirs("gtest")

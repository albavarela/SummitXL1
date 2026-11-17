# CMake generated Testfile for 
# Source directory: /home/ros/catkin_ws/src/people/people_tracking_filter
# Build directory: /home/ros/catkin_ws/build/people_tracking_filter
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_people_tracking_filter_roslaunch-check_launch "/home/ros/catkin_ws/build/people_tracking_filter/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/ros/catkin_ws/build/people_tracking_filter/test_results/people_tracking_filter/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/ros/catkin_ws/build/people_tracking_filter/test_results/people_tracking_filter" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/ros/catkin_ws/build/people_tracking_filter/test_results/people_tracking_filter/roslaunch-check_launch.xml\" \"/home/ros/catkin_ws/src/people/people_tracking_filter/launch\" ")
add_test(_ctest_people_tracking_filter_roslint_package "/home/ros/catkin_ws/build/people_tracking_filter/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/ros/catkin_ws/build/people_tracking_filter/test_results/people_tracking_filter/roslint-people_tracking_filter.xml" "--working-dir" "/home/ros/catkin_ws/build/people_tracking_filter" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/ros/catkin_ws/build/people_tracking_filter/test_results/people_tracking_filter/roslint-people_tracking_filter.xml make roslint_people_tracking_filter")
subdirs("gtest")

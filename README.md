# ROS_QT
contains projects that are done with ros2 and qt

# for qt_ros_robot build
$ colcon build --cmake-args -DCMAKE_PREFIX_PATH=/home/pvsp/Qt/6.9.0/gcc_64 --packages-select qt_ros_robot --cmake-clean-cache

# for running the binary(currently we are not able to run with ros2 run, instead we run it directly)
./install/qt_ros_robot/lib/qt_ros_robot/qt_ros_robot


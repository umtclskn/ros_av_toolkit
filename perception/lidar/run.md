
source /opt/ros/foxy/setup.bash

colcon build --cmake-clean-cache

source ./install/setup.bash


ros2 run ros_av_toolkit perception_lidar_preprocess

ros2 launch ros_av_toolkit perception_lidar.launch.py

ros2 run ros_av_toolkit perception_lidar_bounding_box --ros-args --params-file /home/umut/ros2_example_ws/install/ros_av_toolkit/share/ros_av_toolkit/config/bounding_box_params.yaml

ros2 run ros_av_toolkit perception_lidar_preprocess --ros-args --params-file /home/umut/ros2_example_ws/install/ros_av_toolkit/share/ros_av_toolkit/config/preprocess_params.yaml


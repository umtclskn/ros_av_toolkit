cmake_minimum_required(VERSION 3.5)

project(ros_av_toolkit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(av_toolkit_custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "pcl_conversions"
    "sensor_msgs"
    "av_toolkit_custom_msgs"
    "geometry_msgs"
    "visualization_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
    "EIGEN3"
    "PCL")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_demo_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "PERCEPTION_LIDAR_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "av_toolkit_custom_msgs"
    "pcl_conversions"
    "pcl_msgs"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
    "EIGEN3"
    "PCL")
  target_link_libraries(${library} ${PCL_LIBRARIES})
endfunction()



add_library(perception_lidar_topics_library SHARED
  src/topics/preprocess.cpp
  src/topics/ground_filter.cpp
  src/topics/cluster.cpp
  src/topics/bounding_box.cpp)

add_demo_dependencies(perception_lidar_topics_library)


rclcpp_components_register_node(perception_lidar_topics_library
  PLUGIN "lidar::Preprocess"
  EXECUTABLE perception_lidar_preprocess)
rclcpp_components_register_node(perception_lidar_topics_library
  PLUGIN "lidar::GroundFilter"
  EXECUTABLE perception_lidar_ground_filter)
rclcpp_components_register_node(perception_lidar_topics_library
  PLUGIN "lidar::Cluster"
  EXECUTABLE perception_lidar_cluster)
rclcpp_components_register_node(perception_lidar_topics_library
  PLUGIN "lidar::BoundingBox"
  EXECUTABLE perception_lidar_bounding_box)

install(TARGETS
  perception_lidar_topics_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

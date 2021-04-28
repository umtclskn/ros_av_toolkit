cmake_minimum_required(VERSION 3.5)

project(ros_av_toolkit_perception_camera)

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
find_package(ros_av_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

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
    "ros_av_interfaces"
    "geometry_msgs"
    "visualization_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
    "EIGEN3"
    "PCL"
    "OpenCV")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_camera_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "PERCEPTION_CAMERA_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "av_toolkit_custom_msgs"
    "ros_av_interfaces"
    "pcl_conversions"
    "pcl_msgs"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
    "EIGEN3"
    "PCL"
    "OpenCV")
  target_link_libraries(${library} ${PCL_LIBRARIES} ${OpenCV_LIBS})
endfunction()



add_library(perception_camera_topics_library SHARED
  src/topics/calibrate.cpp
  )

add_camera_dependencies(perception_camera_topics_library)


rclcpp_components_register_node(perception_camera_topics_library
  PLUGIN "camera::Calibrate"
  EXECUTABLE calibrate)


install(TARGETS
  perception_camera_topics_library
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

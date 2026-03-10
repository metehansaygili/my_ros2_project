cmake_minimum_required(VERSION 3.8)
project(gae_sim_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gae_msgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gae_sim_interface_plugin SHARED
  src/GaeSimInterfacePlugin.cpp
)

add_library(traffic_light_plugin SHARED
  src/GazeboTrafficLight.cpp
)

ament_target_dependencies(gae_sim_interface_plugin
  rclcpp
  gazebo_ros
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  gae_msgs
)

ament_target_dependencies(traffic_light_plugin
  rclcpp
  gazebo_ros
  geometry_msgs
  std_msgs
)

target_include_directories(gae_sim_interface_plugin PUBLIC
  include
  $<INSTALL_INTERFACE:include>
)

target_include_directories(traffic_light_plugin PUBLIC
  include
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gae_sim_interface_plugin
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(traffic_light_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
    gae_sim_interface_plugin
    traffic_light_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
    launch
    worlds
    models
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

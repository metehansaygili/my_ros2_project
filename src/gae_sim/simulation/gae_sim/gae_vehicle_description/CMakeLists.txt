cmake_minimum_required(VERSION 3.8)
project(gae_vehicle_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıklar
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 1. C++ GPS Heading Node
add_executable(gps_heading_node src/gps_heading_node.cpp)
target_include_directories(gps_heading_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gps_heading_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2_ros 
  tf2_geometry_msgs
  nav_msgs
)

# 2. Python Teleop Node Kurulumu
# src içindeki python dosyasını çalıştırılabilir olarak kopyalar

# 3. Klasörlerin Kurulumu (Sadece var olanları ekledik)
install(DIRECTORY urdf meshes src
  DESTINATION share/${PROJECT_NAME}
)

# 4. C++ Hedefinin Kurulumu
install(TARGETS
  gps_heading_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
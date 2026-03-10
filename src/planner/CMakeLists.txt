cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# DLIB CONFIGURATION - Based on your working example
# =========================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure dlib build options - Turn OFF ISO_CPP_ONLY to allow threads
set(DLIB_NO_GUI_SUPPORT ON CACHE BOOL "Disable GUI support" FORCE)
set(DLIB_USE_CUDA OFF CACHE BOOL "Disable CUDA" FORCE)
set(DLIB_ISO_CPP_ONLY OFF CACHE BOOL "Allow platform-specific code for threading" FORCE)
set(DLIB_JPEG_SUPPORT OFF CACHE BOOL "Disable JPEG" FORCE)
set(DLIB_PNG_SUPPORT OFF CACHE BOOL "Disable PNG" FORCE)
set(DLIB_GIF_SUPPORT OFF CACHE BOOL "Disable GIF" FORCE)

# Add dlib as a subdirectory
add_subdirectory(Thirdparty/Controller_Lib/dlib-20.0 dlib_build EXCLUDE_FROM_ALL)

include_directories(
  include
  include/${PROJECT_NAME}
  Thirdparty/AStar
  Thirdparty/Clipper2/CPP/Clipper2Lib/include
  Thirdparty/Spline
  Thirdparty/nanoflann/include 
  Thirdparty/Controller_Lib
  Thirdparty/Controller_Lib/dlib-20.0
)

# Clipper2 source files
file(GLOB CLIPPER2_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Clipper2/CPP/Clipper2Lib/src/*.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gae_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Lanelet2
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_io REQUIRED)

# OSQP for MPC
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)

# =========================================================
# LIBRARY DEFINITIONS
# =========================================================

add_library(global_planner_container SHARED src/global_planner.cpp)
add_library(mission_planner_container SHARED src/mission_planner.cpp)
add_library(occupancy_grid_container SHARED
  src/occupancy_grid.cpp
  ${CLIPPER2_SRC}
)
add_library(trajectory_planner_container SHARED src/trajectory_planner.cpp)

# IMPORTANT: Add MPCController.cpp here!
add_library(controller_container SHARED 
  src/controller.cpp
  Thirdparty/Controller_Lib/MPCController.cpp
)

add_library(state_machine_container SHARED src/state_machine.cpp)

# =========================================================
# NODE REGISTRATION
# =========================================================

rclcpp_components_register_node(
    global_planner_container
    PLUGIN "itusct::GlobalPlanner"
    EXECUTABLE global_planner_node
)

rclcpp_components_register_node(
    mission_planner_container
    PLUGIN "itusct::MissionPlanner"
    EXECUTABLE mission_planner_node
)

rclcpp_components_register_node(
    occupancy_grid_container
    PLUGIN "itusct::OccupancyGrid"
    EXECUTABLE occupancy_grid_node
)

rclcpp_components_register_node(
    trajectory_planner_container
    PLUGIN "itusct::TrajectoryPlanner"
    EXECUTABLE trajectory_planner_node
)

rclcpp_components_register_node(
    controller_container
    PLUGIN "itusct::Controller"
    EXECUTABLE controller_node
)

rclcpp_components_register_node(
    state_machine_container
    PLUGIN "itusct::StateMachine"
    EXECUTABLE state_machine_node
)

target_include_directories(mission_planner_container
PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

# =========================================================
# TARGET DEPENDENCIES
# =========================================================

ament_target_dependencies(global_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs gae_msgs lanelet2_core lanelet2_routing lanelet2_projection lanelet2_io tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(mission_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs gae_msgs lanelet2_core lanelet2_routing lanelet2_projection lanelet2_io tf2_geometry_msgs)
ament_target_dependencies(occupancy_grid_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs gae_msgs tf2_ros cv_bridge tf2_geometry_msgs lanelet2_core lanelet2_projection lanelet2_io visualization_msgs) 
ament_target_dependencies(trajectory_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs gae_msgs lanelet2_core lanelet2_projection lanelet2_io tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(controller_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs gae_msgs lanelet2_core lanelet2_routing lanelet2_projection lanelet2_io tf2_ros tf2_geometry_msgs tf2 visualization_msgs osqp_vendor osqp)

# =========================================================
# LINK LIBRARIES AND COMPILE DEFINITIONS
# =========================================================
target_include_directories(controller_container PUBLIC ${EIGEN3_INCLUDE_DIR})

# Add dlib compile definitions to the target (like your working example)
target_compile_definitions(controller_container PRIVATE
    DLIB_NO_GUI_SUPPORT
    DLIB_DISABLE_ASSERTS
)

target_link_libraries(controller_container 
    osqp::osqp
    dlib::dlib
    ${tf2_geometry_msgs_TARGETS}
)

ament_target_dependencies(state_machine_container rclcpp rclcpp_components std_msgs)

# =========================================================
# INSTALLATION
# =========================================================

install(TARGETS 
    global_planner_container 
    mission_planner_container
    occupancy_grid_container
    trajectory_planner_container 
    controller_container
    state_machine_container
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
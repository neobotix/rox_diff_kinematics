cmake_minimum_required(VERSION 3.5)
project(rox_diff_kinematics)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(neo_msgs2 REQUIRED)

# ###########
# ## Build ##
# ###########

ament_export_dependencies(std_msgs tf nav_msgs sensor_msgs)

ament_export_include_directories(common/include)

add_library(${PROJECT_NAME} common/src/DiffDrive2WKinematics.cpp)

ament_export_libraries(${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node ros/src/rox_diff_kinematics.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  neo_msgs2
  trajectory_msgs
  tf2_geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  neo_msgs2
  trajectory_msgs
  tf2_geometry_msgs)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(DIRECTORY common/include/
   DESTINATION common/include/
)

install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  	
  DESTINATION share/${PROJECT_NAME})

ament_package()
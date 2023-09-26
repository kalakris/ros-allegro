cmake_minimum_required(VERSION 3.5)
project(allegro_hand_controllers)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(allegro_hand_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${allegro_hand_driver_INCLUDE_DIRS})

add_library(allegro_node src/allegro_node.cpp)
ament_target_dependencies(
  allegro_node rclcpp sensor_msgs allegro_hand_driver
)


add_executable(allegro_node_pd src/allegro_node_pd.cpp)
target_link_libraries(allegro_node_pd allegro_node)
ament_target_dependencies(allegro_node_pd rclcpp sensor_msgs allegro_hand_driver)

#############
## Install ##
#############

install(TARGETS
  allegro_node
  # allegro_node_grasp
  allegro_node_pd
  # allegro_node_torque
  # allegro_node_velsat
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
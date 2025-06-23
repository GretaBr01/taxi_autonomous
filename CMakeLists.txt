cmake_minimum_required(VERSION 3.5)
project(my_taxi_autonomous)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)


find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  plansys2_msgs
  plansys2_executor
  geometry_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  yaml-cpp
  ros_gz_interfaces
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

# Eseguibili delle azioni

# add_library(${PROJECT_NAME} SHARED src/MoveAction.cpp)
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(drive_normal_action_node src/drive_normal_action_node.cpp)
ament_target_dependencies(drive_normal_action_node ${dependencies})
# target_link_libraries(drive_normal_action_node ${PROJECT_NAME})

add_executable(drive_normal_traffic_action_node src/drive_normal_traffic_action_node.cpp)
ament_target_dependencies(drive_normal_traffic_action_node ${dependencies})
# target_link_libraries(drive_normal_traffic_action_node ${PROJECT_NAME})

add_executable(drive_to_charge_action_node src/drive_to_charge_action_node.cpp)
ament_target_dependencies(drive_to_charge_action_node ${dependencies})
# target_link_libraries(drive_to_charge_action_node ${PROJECT_NAME})

add_executable(drive_to_charge_traffic_action_node src/drive_to_charge_traffic_action_node.cpp)
ament_target_dependencies(drive_to_charge_traffic_action_node ${dependencies})
# target_link_libraries(drive_to_charge_traffic_action_node ${PROJECT_NAME})

add_executable(charge_action_node src/charge_action_node.cpp)
ament_target_dependencies(charge_action_node ${dependencies})

add_executable(pickup_action_node src/pickup_action_node.cpp)
ament_target_dependencies(pickup_action_node ${dependencies})

add_executable(dropoff_action_node src/dropoff_action_node.cpp)
ament_target_dependencies(dropoff_action_node ${dependencies})

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})
target_link_libraries(controller_node yaml-cpp)

add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
ament_target_dependencies(cmd_vel_bridge rclcpp geometry_msgs)

add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher rclcpp geometry_msgs)



# Installazione file launch e PDDL
install(DIRECTORY
  config
  launch
  maps
  params
  pddl
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Installazione binari
install(TARGETS
  # ${PROJECT_NAME}
  drive_normal_action_node
  drive_normal_traffic_action_node
  drive_to_charge_action_node
  drive_to_charge_traffic_action_node
  charge_action_node
  pickup_action_node
  dropoff_action_node 
  controller_node
  cmd_vel_bridge
  cmd_vel_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Testing (opzionale)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})
ament_package()

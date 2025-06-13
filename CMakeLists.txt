cmake_minimum_required(VERSION 3.5)
project(taxi_autonomous)

# Dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
)

# Eseguibili delle azioni
add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(charge_action_node src/charge_action_node.cpp)
ament_target_dependencies(charge_action_node ${dependencies})

add_executable(pickup_action_node src/pickup_action_node.cpp)
ament_target_dependencies(pickup_action_node ${dependencies})

add_executable(dropoff_action_node src/dropoff_action_node.cpp)
ament_target_dependencies(dropoff_action_node ${dependencies})

# Installazione file launch e PDDL
install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

# Installazione binari
install(TARGETS
  move_action_node
  charge_action_node
  pickup_action_node
  dropoff_action_node
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

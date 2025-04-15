cmake_minimum_required(VERSION 3.5)
project(depth_sim)


include_directories(include)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(offboard_node
  src/offboard_node.cpp
  src/stair_follower.cpp      
)


ament_target_dependencies(offboard_node rclcpp px4_msgs std_msgs)


install(TARGETS
  offboard_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

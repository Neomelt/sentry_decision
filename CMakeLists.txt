cmake_minimum_required(VERSION 3.11)
project(rm_decision_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(ros_dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_msgs
  behaviortree_cpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ===== 自定义行为树节点源文件 =====
set(BT_NODE_SOURCES
  src/sentry_decision/topics2blackboard.cpp
  src/sentry_decision/nav2pose.cpp
  src/sentry_decision/chassis_type_publisher.cpp
  src/sentry_decision/attack.cpp
  src/sentry_decision/is_in_position.cpp
)

# 共享库
add_library(${PROJECT_NAME} SHARED
  ${BT_NODE_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

# 可执行文件
add_executable(tree_exec_node
  src/sentry_decision/main.cpp
  ${BT_NODE_SOURCES}
)

target_include_directories(tree_exec_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tree_exec_node ${ros_dependencies})

# ===== 安装 =====
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tree_exec_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
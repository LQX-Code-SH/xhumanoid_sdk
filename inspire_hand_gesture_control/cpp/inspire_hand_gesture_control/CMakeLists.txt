cmake_minimum_required(VERSION 3.8)
project(inspire_hand_gesture_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inspire_hand_msgs REQUIRED)
find_package(inspire_hand_gesture_interfaces REQUIRED)

# 可执行文件
add_executable(hand_gesture_control_node src/hand_gesture_control.cpp)

target_include_directories(hand_gesture_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hand_gesture_control_node
  rclcpp
  std_msgs
  inspire_hand_msgs
  inspire_hand_gesture_interfaces
)

# 安装目标
install(TARGETS hand_gesture_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

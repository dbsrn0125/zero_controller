cmake_minimum_required(VERSION 3.8)
project(zero_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zenith_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED) # ✅ 1. find_package 추가

add_library(bt_action_nodes SHARED
  src/bt_action_nodes.cpp
)
target_include_directories(bt_action_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_action_nodes
  rclcpp
  behaviortree_cpp
  lifecycle_msgs # ✅ 2. 라이브러리 의존성에 추가
)

add_executable(zero_controller_node src/zero_controller_node.cpp)
target_include_directories(zero_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  zero_controller_node
  rclcpp
  zenith_interfaces
  behaviortree_cpp
  ament_index_cpp
  # 실행 파일은 bt_action_nodes 라이브러리를 통해 간접적으로 의존하므로
  # 여기에 직접 추가할 필요는 없습니다.
)
target_link_libraries(zero_controller_node bt_action_nodes)

install(
  TARGETS zero_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS bt_action_nodes
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8...3.10)
project(teamx_challenge)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(pose_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

add_executable(vision_node src/vision_node.cpp)
add_executable(shooter_node src/shooter_node.cpp)


ament_target_dependencies(vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
ament_target_dependencies(shooter_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(vision_node ${OpenCV_LIBS})
target_link_libraries(shooter_node ${OpenCV_LIBS})


# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(vision_node ${typesupport_target})
target_link_libraries(shooter_node ${typesupport_target})
# 安装Vision_test
install(TARGETS 
vision_node
shooter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY numbers/
  DESTINATION share/${PROJECT_NAME}/numbers
)
# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})


ament_package()

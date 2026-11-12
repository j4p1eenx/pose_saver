cmake_minimum_required(VERSION 3.8)
project(pose_saver_pkg)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(cloud_db_client STATIC src/cloud_db_client.cpp)
target_include_directories(cloud_db_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(cloud_db_client PUBLIC nlohmann_json::nlohmann_json CURL::libcurl)

add_executable(pose_saver_node src/pose_saver_node.cpp)
target_link_libraries(pose_saver_node cloud_db_client)
ament_target_dependencies(pose_saver_node rclcpp geometry_msgs nav_msgs
  sensor_msgs std_msgs std_srvs tf2_ros)


add_executable(pose_crud_cli src/pose_crud_cli.cpp)
target_link_libraries(pose_crud_cli cloud_db_client)
ament_target_dependencies(pose_crud_cli nav_msgs sensor_msgs)

install(TARGETS pose_saver_node pose_crud_cli DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_logic test/test_pose_logic.cpp)
  target_link_libraries(test_pose_logic cloud_db_client)
  ament_target_dependencies(test_pose_logic nav_msgs sensor_msgs)
endif()

ament_package()

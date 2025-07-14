cmake_minimum_required(VERSION 3.8)
project(gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components   REQUIRED)
find_package(tosor_msgs REQUIRED)                      # CHANGE
find_package(Eigen3 REQUIRED)

add_subdirectory(third_party/serial)

add_executable(dh_gripper src/dh_gripper.cpp)
target_link_libraries(dh_gripper  ${catkin_LIBRARIES}   Eigen3::Eigen serial )
ament_target_dependencies(dh_gripper rclcpp tosor_msgs rclcpp_action rclcpp_components)    # CHANGE
add_executable(test_gripper src/test_gripper.cpp)
target_link_libraries(test_gripper  ${catkin_LIBRARIES}   Eigen3::Eigen serial )
ament_target_dependencies(test_gripper rclcpp tosor_msgs rclcpp_action rclcpp_components)    # CHANGE

# add_executable(listener src/subscriber_member_function.cpp)
# ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE


install(
        DIRECTORY  config launch meshes urdf
        DESTINATION share/${PROJECT_NAME}
)

#
install(TARGETS
        dh_gripper

        # listener
        DESTINATION lib/${PROJECT_NAME})

ament_package()

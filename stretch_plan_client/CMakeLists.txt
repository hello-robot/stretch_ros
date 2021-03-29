cmake_minimum_required(VERSION 3.5)
project(stretch_plan_client)

find_package(Eigen3)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(stretch_plan_client src/stretch_plan_client.cpp)
ament_target_dependencies(stretch_plan_client rclcpp moveit_ros_planning_interface)
ament_export_dependencies(stretch_plan_client moveit_ros_planning_interface rclcpp)
install(TARGETS stretch_plan_client
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
ament_package()

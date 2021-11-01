cmake_minimum_required(VERSION 3.0.2)
project(stretch_gazebo)

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        roscpp
        std_msgs
        control_toolbox
        controller_manager
        hardware_interface
        transmission_interface
        pluginlib
        joint_limits_interface
        urdf
        angles)

catkin_package(INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_ros_control_plugin ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        controller_manager
        control_toolbox
        pluginlib
        hardware_interface
        transmission_interface
        joint_limits_interface
        urdf
        angles
        hardware_interface)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(${PROJECT_NAME}_ros_control_plugin src/stretch_hardware_gazebo_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_ros_control_plugin ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/stretch_hardware_gazebo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


install(DIRECTORY config launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}_ros_control_plugin ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
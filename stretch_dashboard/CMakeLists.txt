cmake_minimum_required(VERSION 3.0.2)
project(stretch_dashboard)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/dashboard
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

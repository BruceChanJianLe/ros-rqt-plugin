cmake_minimum_required(VERSION 3.0.2)
project(ros_rqt_plugin)

find_package(catkin REQUIRED COMPONENTS
  # geometry_msgs
  # roscpp
  # rospy
  # std_msgs
  # rqt_gui
  # rqt_gui_cpp
  # rqt_gui_py
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES ${PROJECT_NAME}
  #  CATKIN_DEPENDS roscpp rospy std_msgs rqt_gui rqt_gui_cpp rqt_gui_py
  #  DEPENDS #system_lib
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# Install plugin path
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
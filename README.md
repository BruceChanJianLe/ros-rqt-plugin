# ROS RQt Plugin

This repository demonstrates the usage of using ROS rqt plugin to design personal UI interface with both cpp and python.

## Content
- [Content](#Content)
- [Package Creation](#Package-Creation)
  - [Dependencies](#Dependencies)
  - [Package Folder Structure](#Package-Folder-Structure)
- [UI File](#UI-File)
- [Reference](#Reference)
- [CMakeLists.txt](#CMakeListstxt)

## Package Creation

Create your personal ROS RQt plugin with the following command:
```bash
catkin_create_pkg ros_rqt_plugin roscpp rospy rqt_gui rqt_gui_cpp rqt_gui_py std_msgs 
```
### Dependencies

To create a RQt plugin these are the dependencies that you actually need, the other dependencies included along with the catkin_create_pkg is for this demonstration purpose.  
```bash
roscpp
rospy
rqt_gui
rqt_gui_cpp
rqt_gui_py
```

### Package Folder Structure
```bash
ros_rqt_plugin
├── CMakeLists.txt
├── package.xml
├── plugin.xml
├── include
│   └── ros_rqt_plugin
│       └── ros_rqt_plugin.hpp
└── src
    └── ros_rqt_plugin
            ├── ros_rqt_plugin.cpp
            └── ros_rqt_plugin.ui
```
## UI File

Create your ui file with `Qt 5 Designer`.
For more information, please look at this [video](https://www.youtube.com/watch?v=2mIyZX6x-S0).

## CMakeLists.txt

For the convienience of different Qt version, I have followed the exampl from `rqt_image_viewer`.

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(ros_rqt_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  rqt_gui
  rqt_gui_cpp
  rqt_gui_py
)

# Find qt package
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

# Define source file
set(${PROJECT_NAME}_SRCS
  src/ros_rqt_plugin/ros_rqt_plugin.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/ros_rqt_plugin/ros_rqt_plugin.hpp
)

# Define include directory
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ros_rqt_plugin
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs rqt_gui rqt_gui_cpp rqt_gui_py
#  DEPENDS system_lib
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# Obtain qt wrap cpp
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${{PROJECT_NAME}_HDRS})
endif()

# Obtain qt wrap ui
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${{PROJECT_NAME}_UIS})
else()
  qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${{PROJECT_NAME}_UIS})
endif()

# Ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}"

include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_MOCS} ${${PROJECT_NAME}_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

# Find class loader
# For more information please visit: http://wiki.ros.org/class_loader
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# Install plugin path
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/ros_rqt_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

## Reference

- Video lesson on UI file. [link](https://www.youtube.com/watch?v=2mIyZX6x-S0)
- updated rqt_image_viewer. [link](https://github.com/ros-visualization/rqt_image_view)
- older rqt_image_viewer. [link](https://github.com/ros-visualization/rqt_common_plugins/tree/619481f13084db01c7e2233b754bb0409ce1c44a/rqt_image_view)

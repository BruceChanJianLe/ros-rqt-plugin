# ROS RQt Plugin

This repository demonstrates the usage of using ROS rqt plugin to design personal UI interface with both cpp and python.

## Content
- [Content](#Content)
- [Package Creation](#Package-Creation)
  - [Dependencies](#Dependencies)
  - [Package Folder Structure](#Package-Folder-Structure)
- [UI File](#UI-File)
- [Reference](#Reference)

## Package Creation

Create your personal ROS RQt plugin with the following command:
```bash
catkin_create_pkg pose_recorder roscpp rospy rqt_gui rqt_gui_cpp rqt_gui_py std_msgs 
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

## Reference

- Video lesson on UI file. [link](https://www.youtube.com/watch?v=2mIyZX6x-S0)

# QT_Gripper

## Description

This program calculates the Forward and Inverse Kinematics of a 3Dof Manipulator using QT with Cmake and QChart. 
It's able to generate the path to write the number 19044686, to complete this operation follow the steps:

1. Click the "Write Number" button on the kinematics tab to generate the coordinates using the inverse kinematics.

2. Click the "Draw Number" button to generate the graphics. The graphics will be displayed at the Graphics tab.

The OpenGL was not fully implemented, since its not subject of this assignment. It will be implemented in the future to simulate the solidworks parts.


## libraries

The Eigen library is imported directly from the folder, no CMake required.

Qt5::Charts and Qt5::SerialPort as well as ${OPENGL_LIBRARIES} are imported at the CMake file.



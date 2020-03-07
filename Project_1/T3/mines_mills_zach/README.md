# Project 1 Task 3

My package was made inside a catkin workspace so running `catkin_make` in such a workspace will build it

The python script executable requires the rospkg python library in order to properly navigate the directory structure of the catkin workspace, and find the coordinate data file in the *data* directory.

After building the package, calling `rosrun mines_mills_zach mines_mills_zach.py` will cause the script to begin publishing to the /turtle1/cmd_vel topic, so turtlesim must be running

`outputs/example.png` shows an example of the output of the package

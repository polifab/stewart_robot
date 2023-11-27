#!/bin/bash                                                                     
ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
                                                             
echo 'Building Stewart Controller'

source /opt/ros/noetic/setup.bash 
cd /workspace/controller_ws
catkin_make

wait
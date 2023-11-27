#!/bin/bash                                                                     

source /opt/ros/noetic/setup.bash  && \
source /workspace/controller_ws/devel/setup.bash && \
roslaunch stewart_controller stewart.launch
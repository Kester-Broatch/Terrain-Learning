FROM osrf/ros:melodic-desktop-full
LABEL Description="Terrain-Learning" Maintainer="Kester Broatch"

# Setup catkin workspace
# RUN mkdir -p /catkin_ws/src
COPY /catkin_ws/src /catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make' 

# Modified ros_entrypoint file to add catkin WS
COPY /docker/ros_entrypoint.sh /ros_entrypoint.sh

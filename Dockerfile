# pull container from https://hub.docker.com/_/ros, choosing the melodic-robot-bionic version
FROM ros:melodic-robot-bionic

# install dependencies for the F1Tenth simulator and wget
RUN sudo apt-get update
RUN sudo apt-get -y install ros-melodic-ackermann-msgs ros-melodic-tf2-geometry-msgs ros-melodic-interactive-markers ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-rviz ros-melodic-joy ros-melodic-map-server wget vim
# switch to bash for running commands
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# initialise ROS, pull useful scripts from our repo and clone the F1Tenth simulator into a new catkin workspace
RUN source /ros_entrypoint.sh && \
    source /opt/ros/melodic/setup.bash && \
    mkdir f110 && \
    cd f110 && \
    git clone https://github.com/FT-Autonomous/f1tenth-ros-setup.git && \
    mv f1tenth-ros-setup/utils /utils && \
    rm -rf f1tenth-ros-setup && \
    mkdir f110_workspace && \
    cd f110_workspace && \
    git clone https://github.com/f1tenth/f110_ros.git src
# remove extra skeleton code included in f110_ros repo
RUN find /f110/f110_workspace/src -mindepth 1 ! -regex '^/f110/f110_workspace/src/f110_simulator\(/.*\)?' -delete

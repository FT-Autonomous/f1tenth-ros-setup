# pull container from https://hub.docker.com/_/ros, choosing the melodic-robot-bionic version
FROM ros:melodic-robot-bionic

# Copy in utils scripts
COPY ./utils /utils

# install dependencies for the F1Tenth simulator and wget
RUN sudo apt-get update
RUN sudo apt-get -y install ros-melodic-ackermann-msgs ros-melodic-tf2-geometry-msgs ros-melodic-interactive-markers ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-rviz ros-melodic-joy ros-melodic-map-server wget vim
# switch to bash for running commands
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# initialise ROS, pull useful scripts from our repo and clone the F1Tenth simulator into a new catkin workspace
RUN source /ros_entrypoint.sh && \
    source /opt/ros/melodic/setup.bash && \
    mkdir -p f1tenth_workspace/src && \
    cd f1tenth_workspace/src && \
    git clone https://github.com/f1tenth/f1tenth_simulator.git && \
    cd f1tenth_simulator/maps && \
    git init && \
    git remote add racetracks https://github.com/f1tenth/f1tenth_racetracks && \
    git pull racetracks main

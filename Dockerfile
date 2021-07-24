# pull container from https://hub.docker.com/_/ros, choosing the melodic-robot-bionic version
FROM ros:melodic-robot-bionic

# install dependencies for the F1Tenth simulator and wget (and now pip)
RUN sudo apt-get update
RUN pip3 install -U pip

RUN sudo apt-get -y install ros-melodic-ackermann-msgs ros-melodic-tf2-geometry-msgs ros-melodic-interactive-markers ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-rviz ros-melodic-joy ros-melodic-map-server wget vim python3-pip
# switch to bash for running commands
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# initialise ROS, pull useful scripts from our repo and clone the F1Tenth simulator into a new catkin workspace
RUN source /ros_entrypoint.sh && \
    source /opt/ros/melodic/setup.bash && \
    mkdir f1tenth && \
    cd f1tenth && \
    git clone https://github.com/FT-Autonomous/f1tenth-ros-setup.git && \
    mv f1tenth-ros-setup/utils /utils && \
    rm -rf f1tenth-ros-setup && \
    mkdir -p f1tenth_workspace/src && \
    cd f1tenth_workspace/src && \
    git clone https://github.com/f1tenth/f1tenth_simulator.git

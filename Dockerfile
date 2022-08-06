# pull container from https://hub.docker.com/_/ros, choosing the melodic-robot-bionic version
FROM ros:melodic-robot-bionic

# install dependencies for the F1Tenth simulator and wget (and now pip)
RUN sudo apt update

RUN sudo apt install -y ros-melodic-ackermann-msgs ros-melodic-tf2-geometry-msgs ros-melodic-interactive-markers ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-rviz ros-melodic-joy ros-melodic-map-server wget neovim python3-pip netcat x11vnc xvfb xorg-dev
# switch to bash for running commands

# mightn't be necessary
RUN pip3 install -U pip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# initialise ROS, pull useful scripts from our repo and clone the F1Tenth simulator into a new catkin workspace
RUN git clone https://git.suckless.org/st/ && cd st && make install
RUN git clone https://github.com/FT-Autonomous/dwm && cd dwm && make install

RUN source /ros_entrypoint.sh && \
    git clone https://github.com/novnc/noVNC.git /noVNC && \
    source /opt/ros/melodic/setup.bash && \
    git clone https://github.com/FT-Autonomous/f1tenth-ros-setup.git && \
    mv f1tenth-ros-setup/utils /utils && \
    rm -rf f1tenth-ros-setup && \
    mkdir -p f1tenth_workspace/src && \
    cd f1tenth_workspace/src && \
    git clone https://github.com/f1tenth/f1tenth_simulator.git && \
    cd f1tenth_simulator/maps && \
    git init && \
    git remote add racetracks https://github.com/f1tenth/f1tenth_racetracks && \
    git pull racetracks main

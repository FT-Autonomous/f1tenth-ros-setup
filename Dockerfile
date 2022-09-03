# pull container from https://hub.docker.com/_/ros, choosing the melodic-robot-bionic version
FROM ros:noetic
# install dependencies for the F1Tenth simulator and wget (and now pip)
RUN apt update && apt install -y ros-noetic-xacro ros-noetic-ackermann-msgs ros-noetic-tf2-geometry-msgs ros-noetic-interactive-markers ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-rviz ros-noetic-joy ros-noetic-map-server wget neovim python3-pip x11vnc xvfb xorg-dev git python3-numpy
# Initialise code for geting graphical output from the docker container
RUN git clone https://git.suckless.org/st/ && cd st && make clean install ; \
    git clone https://github.com/FT-Autonomous/dwm && cd dwm && make clean install ; \
    git clone https://github.com/novnc/noVNC.git /noVNC ; \
    git clone https://github.com/novnc/websockify /noVNC/utils/websockify
# Pull in utility scripts
COPY ./utils /utils
RUN cp -f /bin/bash /bin/sh
RUN /utils/setup-workspace.sh /f1tenth_workspace

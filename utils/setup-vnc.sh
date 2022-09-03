#!/bin/sh
export SCREEN=0 DISPLAY=:0 WINDOW_MANAGER=dwm
source /ros_entrypoint.sh
pgrep Xvfb || (rm -fr /tmp/.X* && Xvfb "$DISPLAY" -screen $SCREEN 1280x720x16 -ac -pn -noreset) & sleep 1
pgrep $WINDOW_MANAGER || nohup $WINDOW_MANAGER > /tmp/${WINDOW_MANAGER}.log &
pgrep x11vnc || x11vnc -forever -noxdamage -noxrecord -display $DISPLAY -passwd f1tenth &
pgrep novnc_proxy || ./noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080

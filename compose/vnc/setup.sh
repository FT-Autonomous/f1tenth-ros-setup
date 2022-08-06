export DISPLAY=:0 WINDOW_MANAGER=dwm
sleep 1
Xvfb "$DISPLAY" -screen 0 1440x900x16 -ac -pn -noreset & 2> /dev/null
sleep 1
nohup $WINDOW_MANAGER >/tmp/${WINDOW_MANAGER}.log & 2> /dev/null
sleep 1
x11vnc -display $DISPLAY &
sleep 1
./noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080

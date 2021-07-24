cd /f1tenth/f1tenth_workspace
catkin_make
source devel/setup.bash
if [ $1 ]
then
	roslaunch f1tenth_simulator simulator.launch map:=/f1tenth/f1tenth_workspace/src/f1tenth_simulator/maps/$1/$1_map.yaml
else
	roslaunch f1tenth_simulator simulator.launch
fi

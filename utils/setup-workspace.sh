workspace_dir="${1-f1tenth_shared_workspace}/src"
mkdir -p "$workspace_dir"
cd "$workspace_dir"
git clone https://github.com/f1tenth/f1tenth_simulator.git
cd f1tenth_simulator/maps
git init && git pull https://github.com/f1tenth/f1tenth_racetracks

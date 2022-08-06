# F1Tenth ROS Setup

A simplified setup and workspace for using F1Tenth with ROS and Docker.

## Prerequisites

### Windows/Mac
* [git](https://www.atlassian.com/git/tutorials/install-git)
* [Docker Desktop](https://www.docker.com/products/docker-desktop)
* [Docker Compose](https://docs.docker.com/compose/install/)

### Editor 
* [Visual Studio Code](https://www.toolsqa.com/blogs/install-visual-studio-code/) (unless you have some other preference)

### Linux
* Make sure to sync packages before installing new packages
* [git](https://www.atlassian.com/git/tutorials/install-git)
    * Debian based
        * `sudo apt update` (syncing packages)
        * `sudo apt install git`
    * Arch based
        * `sudo pacman -Sy` (syncing packages)
        * `sudo pacman -S git` 
    * Gentoo based
        * `sudo emerge --sync` (syncing packages)
        * `sudo emerge git`
    
* [Docker](https://www.docker.com/products/docker-desktop) 
    * [Debian based](https://docs.docker.com/engine/install/ubuntu/)
        * See [here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-compose-on-ubuntu-20-04) for instructions on how to install docker compose on ubuntu.
    * [Arch based](https://wiki.archlinux.org/title/Docker) 
        * `sudo pacman -S yay base-devel`
        * `yay -S docker-git docker-compose`
    * [Gentoo based](https://wiki.gentoo.org/wiki/Docker) 
        * `sudo emerge app-containers/docker app-containers/docker-cli app-containers/docker-compose`

#### Systemd based - Debian/Arch etc
* `sudo systemctl enable docker`
* `sudo systemctl start docker`

#### Init Systems 

##### OpenRC - Gentoo
* `sudo rc-update add docker`
* `sudo rc-service docker start`
* If encountering a crash from docker, manually solve it by `sudo rc-service docker zap`

#### User permisions
* `sudo usermod -aG docker <username>` 

## Building the Docker Image and making a Container

These steps should only have to be done once. Run the commands in the following steps in a Terminal.

#### 1. Clone this repository

```
git clone https://github.com/FT-Autonomous/f1tenth-ros-setup.git
```

#### 2. Build the Docker Image

The build might take a while to load the first time you run it.

```
cd f1tenth-ros-setup
docker compose build
```

#### 4. Run a Container

This step will ensure the build has worked by running a container. Later we will run the simulator in this container.

```
docker compose run f1tenth-ros
```

You should see your terminal display something like

```
root@...:/#
```

You may now exit the container by pressing CTRL+D. From now on we will be running this container from VSCode.

## Running the Container in VSCode

You will do this anytime you want to run the simulator or work on your solution.

#### 1. Ensure you have the `Remote - Containers` extension installed in VSCode

Extensions (in Sidebar on left of screen) -> Search `Remote - Containers` -> Click the one with the star -> Install

#### 2. Using Remote Explorer to attach to the container

Remote Exporer (in Sidebar on left of screen) -> Ensure the dropdown at the top is set to `Containers` -> there should be an `f1tenth-ros` container shown (hit refresh button if not) -> right-click `f1tenth-ros` -> Attach to Container


This will start the container we made previously and then attach to it (allow us to edit code and run commands), opening it in a new window.

#### 3. Opening the Workspace

After the previous step, you should see an `Open Folder` button. Click it and type the path `/f1tenth_workspace/`.

It may take a few seconds to load.

You should see a `src` folder. This holds the `f1tenth_simulator` package and will also be the place we create our own packages.

You should open a terminal now, as we will need at least one to run the simulator and our code: Terminal (top taskbar) -> New Terminal.

You can see that a terminal opens, with the current directory being our workspace.

Each time you open a Terminal, you should run the `ros-init` script, as this will ensure you are able to use the ROS commands.

```
source /utils/ros-init.sh
```

## Running the F1Tenth Simulator

When inside the running container, you may execute the `run-simulator` script from the `utils` folder. This will build the workspace, then use `roslaunch` to launch the simulator. As you get more used to ROS, you may not want to build the workspace each time, but for now, this is a foolproof way of getting the simulator running.

```
source /utils/run-simulator.sh
```

If you don't have display access set up yet you will see something like `[rviz-X] process has died...`. Don't worry, the simulator is still running, it just has no display output. See the end of this README to set up display access.

## Choosing a racetrack

All of the racetracks in the [F1Tenth Racetracks Repo](https://github.com/f1tenth/f1tenth_racetracks) can be used in the simulator. To choose a racetrack, you add the name of the track as a parameter in the `run-simulator` command, e.g. to run the simulator using Silverstone as the racetrack:

```
source /utils/run-simulator.sh Silverstone
```

## Adding your own packages

Clone your package into the `src` directory. If you are in FTA, you will clone the [FTA F1Tenth Driver](https://github.com/FT-Autonomous/fta_f1tenth_driver.git) in here.

Ensure you do a build of the workspace after you add a package so that ROS can find it:

```
cd /f1tenth_workspace
catkin_make
source devel/setup.bash
```

## Making nodes launch whenever the Simulator launches

Makes testing smoother, as you can run everything from one terminal. This is best shown with an example.

In this case, you have created a package called `my_package` which contains a source file `driver.py`. To launch `driver.py` whenever the simulator launches, you would add the following line to the `simulator.launch` file:

```launch
<launch>
...
<node pkg="my_package" name="disparity_node" type="driver.py" output="screen"/>
</launch>
```

Note that the `name` field corresponds to what you decide to name the node when initializing it using `rospy.init_node` in the `driver.py` file.

## Running nodes seperately while the simulator is running

Use the plus button in the Terminal window to add another terminal. 

Run the usual initialization script as described above

```
source /utils/ros-init.sh
```

Ensure you are using the environment created when the workspace was last built

```
source /f1tenth_workspace/devel/setup.bash
```

Run a file in your package

```
rosrun <package-name> <filename>
```

### Stuff to note about running packages:

* If you get an unexpected `Couldn't find executable named...`, you may need to make your file executable:

```
chmod +x <filepath>
```

* If you get something like `Unable to register with master node`, the simulator isn't running. Ensure the simulator is running before running your own packages.

* Having multiple terminals open can get confusing, luckily in VSCode you can rename each terminal to something more convenient: Right-click terminal name -> rename.

## Enabling the container to display GUIs on the host machine

```
docker compose run -p 6080:6080 f1tenth-ros sh setup.sh
```

Then open `localhost:6080/vnc.html` in your browser.

[DWM Keybinding](https://github.com/FT-Autonomous/dwm) (keybindings for general use in the docker container)

Open a terminal and then type in `source /utils/run-simulator.sh Silverstone`.

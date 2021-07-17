# F1Tenth ROS Setup

A simplified setup and workspace for using F1Tenth with ROS and Docker.

## Prerequisites

* [git](https://www.atlassian.com/git/tutorials/install-git)
* [Docker Desktop](https://www.docker.com/products/docker-desktop)
* [Visual Studio Code](https://www.toolsqa.com/blogs/install-visual-studio-code/) (unless you have some other preference)

## Building the Docker Image and making a Container

These steps should only have to be done once. Run the commands in the following steps in a Terminal.

#### 1. Clone this repository

```
git clone https://github.com/FT-Autonomous/f110-ros-setup.git
```

#### 2. Build the Docker Image

The build might take a while to load the first time you run it.

```
cd f110-ros-setup
docker build -t f110-ros .
```

#### 4. Run a Container

This step will ensure the build has worked by running a container. Later we will run the simulator in this container.

```
docker run -it f110-ros
```

You should see your terminal display something like

```
root@...:/#
```

If it's all good, you may now exit the container by pressing CTRL+D. From now on we will be running this container from VSCode.

## Running the Container in VSCode

You will do this anytime you want to run the simulator or work on your solution.

#### 1. Ensure you have the `Remote - Containers` extension installed in VSCode

Extensions (in Sidebar on left of screen) -> Search `Remote - Containers` -> Click the one with the star -> Install

#### 2. Using Remote Explorer to attach to the container

Remote Exporer (in Sidebar on left of screen) -> Ensure the dropdown at the top is set to `Containers` -> there should be an `f110-ros` container shown (hit refresh button if not) -> right-click `f110-ros` -> Attach to Container


This will start the container we made previously and then attach to it (allow us to edit code and run commands), opening it in a new window.

#### 3. Opening the Workspace

After the previous step, you should see an `Open Folder` button. Click it and type the path `/f110/f110_workspace/`.

It may take a few seconds to load.

You should see a `src` folder. This holds the `f110_simulator` package and will also be the place we create our own packages.

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

## Adding your own packages

Clone your package into the `src` directory. If you are in FTA, you will clone the [FTA F1Tenth Driver](https://github.com/FT-Autonomous/fta_f1tenth_driver.git) in here.

Ensure you do a build of the workspace after you add a package so that ROS can find it:

```
cd /f110/f110_workspace
catkin_make
source devel/setup.bash
```

## Running other packages while the simulator is running

Use the plus button in the Terminal window to add another terminal. 

Run the usual initialization script as described above

```
source /utils/ros-init.sh
```

Ensure you are using the environment created when the workspace was last built

```
source /f110/f110_workspace/devel/setup.bash
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

## Enabling the container to display GUIs on the host machine (Windows)

Prerequisite: Chocolatey must be installed on your computer.

Warning: Some firewalls may block access so it might be necessary to disable your firewall when running the simulator.

Open Windows Powershell as Administrator and install the `xming` package

```
choco install xming
```

Open the XLaunch Application

<img src="https://i.imgur.com/q6bDsQV.jpg" width="300">

Follow the default settings for each step except the following

<img src="https://i.imgur.com/zn3YArf.jpg" width="400">

Once the XLaunch setup is finished, find your local IP Address by entering an ipconfig command in your local terminal

```
ipconfig
```

Your local IP should be found similarly to where its shown below (ignore the white marks)

<img src="https://i.imgur.com/JsNgYMO.jpg" width="400">

Execute the setup-display script in the container passing your local IP Address as a parameter.

```
source /utils/set-display.sh <Local IP Address>
```

Now when you run the simulator you should see the display open in a new window.

### Enabling the Container to display GUIs on the host machine (Ubuntu, and other)

First, install [Rocker](https://github.com/osrf/rocker), then after you have built your image (step 3), run the following command in your Terminal to run the Docker image:

```
rocker [optional: --nvidia or --devices /dev/dri/card0] --x11 docker-ros
```

# Mobile Robot Simulation Biorobotics UNAM


![GUI](https://raw.githubusercontent.com/dieg4231/MobileRobotSimulator/master/screenshot.png)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

The things you need to install:

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Ubuntu 16.4](http://releases.ubuntu.com/16.04/)


### Installing

- Clone this repo
```
git  clone https://github.com/dieg4231/MobileRobotSimulator.git

```


- Go to folder catkin_ws

```
cd MobileRobotSimulator/catkin_ws

```

- run the script install.sh

```
./install.sh
```

and then

```
catkin_make
```


### Run

- The source command can be used to load any functions file into the current shell script or a command prompt, in this case the file catkin_ws/devel/setup.bash, so in the folder catkin_ws execute:
```
source devel/setup.bash

```


- Launch the simulator:

```
 roslaunch simulator simulator.launch 

```

- Or execute start.sh (this option open a xterminal for each node)

```
./start.sh
```


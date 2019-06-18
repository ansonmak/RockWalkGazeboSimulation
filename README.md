# Rocking and Walking Manipulation with Gazebo Simulator
This simulation is made to reproduce the experiment of Rocking and Walking Manipulation for transporting cone object. For more details about the manipulation, see [PRW-Manipulation](https://github.com/HKUST-RML/PRW-Manipulation) for manuscript and demo video.
## Usage
This repository is a workspace that contains `universal_robot` ROS package with modified launch files to perform simulation for Rocking and Walking Manipulation by Gazebo. The UR10 robot arm with customized caging end effector represented by Unified Robot Description Format (URDF) will be loaded into the simulated world as the launch file launched. The UR10 robot arm is controlled by a ROS node through `MoveIt!` to perform manipulation. The simulation was tested on **Ubuntu 16.04 LTS**.

## Getting Started

### Prerequisites
- ROS Kinetic
- MoveIt!
- Gazebo 7 (Should be included with ROS Kinetic desktop-full installation)
- catkin_tools

### Setting up the workspace
1. Clone this repository into your home directory, and rename it to `ur_ws`
```bash
$ git clone https://github.com/ansonmak/RWM-Simulation.git
```
2. Enter the `ur_ws` directory and run the following command to build the workspace
```bash
$ cd ws_moveit
$ catkin init
$ catkin clean
$ catkin build
```
3.Source the setup.bash to your bash session every time a new terminal is launched
```bash
$ source ~/ur_ws/devel/setup.bash
```
or run the following command to add it in the `.bashrc` for executing it everytime when a new terminal is launched
```bash
$ echo "source ~/ur_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Launching the simulation

## Author
KaHei Mak (khmakac@connect.ust.hk)


# Rocking and Walking Manipulation with Gazebo Simulator
This simulation is made to reproduce the experiment of Rocking and Walking Manipulation for transporting cone object. For more details about the manipulation, see [PRW-Manipulation](https://github.com/HKUST-RML/PRW-Manipulation) for manuscript and demo video.
## Usage
This repository is a workspace that contains `universal_robot` ROS package with modified launch files to perform simulation for Rocking and Walking Manipulation by Gazebo. The UR10 robot arm with customized caging end effector represented by Unified Robot Description Format (URDF) will be loaded into the simulated world as the launch file launched. The UR10 robot arm is controlled by a ROS node through `MoveIt!` to perform manipulation. The simulation was tested on **Ubuntu 16.04 LTS**.

## Get Started

### Prerequisites
- ROS Kinetic
- MoveIt!
- Gazebo 7 (Will be installed with ROS Kinetic desktop-full installation)

### Setting up the workspace

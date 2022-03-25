# ViV ROS

Ros packages for the ViV UGV developed for the [HEKTOR](https://hektor.fer.hr/) project.

Contains description, control and Gazebo packages. 

![viv1](doc/viv_pequeno_img.png)

## Installation

### Dependencies
None
### Build

Clone into a catkin workspace and build with

	catkin build viv_gazebo

## Usage

### Launching simulation

To launch an empty world simulation

	roslaunch viv_gazebo viv_empty_world.launch  

To launch a simulator with a randomly generated vineyard:

1. Navigate to viv_description/scripts

2. Generate a spawn_random_vineyard.launch with generate_random_vineyard_launch 
	
	Example:

		./generate_random_vineyard_launch.py 5 2  

	to generate a spawn_random_vineyard.launch with 2 rows, and 5 randomly selected plant models in a single row

3. Launch the simulation with:

		roslaunch viv_gazebo viv_random_vineyard.launch  


### Control
ViV is spawned with a differential drive controller 

Velocity is commanded by publishing a Twist message to 
   
    /viv/velocity_controller/cmd_vel

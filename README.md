# ViV ROS

Ros packages for the ViV UGV developed for the [HEKTOR](https://hektor.fer.hr/) project.

Contains description, control and Gazebo packages. 

![viv1](doc/viv_pequeno_img.png)

## 📄 Dependencies

This project depends on ROS

## 🛠️ Usage

### ⚙️ Build the project

Clone into a catkin workspace and build with

	catkin build viv_gazebo

### 🖥️ Launching the simulation

- To launch an empty world simulation:

		roslaunch viv_gazebo viv_empty_world.launch  

- To launch a simulator with a randomly generated vineyard:

	1. Navigate to viv_description/scripts

	2. Once you are located in viv_description/scripts generate a spawn_random_vineyard.launch with generate_random_vineyard_launch 
	
		- Example:

			- to generate a spawn_random_vineyard.launch with 2 rows, and 5 randomly selected plant models in a single row:

					./generate_random_vineyard_launch.py 5 2  


	3. Launch the simulation with:

			roslaunch viv_gazebo viv_random_vineyard.launch spawn_d435:=true

	4. Wait for all the grapevines to spawn, and unpause the simulation


### 🚜 Control
ViV is spawned with a ROS [differential drive controller](http://wiki.ros.org/diff_drive_controller) 

- Velocity is commanded by publishing a Twist message to 
   
    	/viv/velocity_controller/cmd_vel

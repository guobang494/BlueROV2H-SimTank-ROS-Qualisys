# Simulation demo instructions
These instructions start a Gazebo demo to run the BlueROV2H controlled via MPC, simulated in the UCL Ocean Towing Tank.   
If you want to change the path scenario, we provide further instructions in the ![Path scenario instructions](./Path_scenario_instructions.md/) file.  

### 0) Pre-requisite
These instructions assume that you have installed the provided Docker container, as explained in ![INSTALLATION](./INSTALLATION.md/) file.     



### 1) Start the docker container
If you do not have any terminal session ongoing, open a new terminal and type:
```
sudo docker start -ai bluerov2h_container
```

If you are following up directly from the ![INSTALLATION](./INSTALLATION.md/) instruction file, your container might be already open.   
In that case, use the previous terminal, or open a new terminal and run instead:
```
sudo docker exec -it bluerov2h_container bash
```


### 2) Set container display (not in the Docker)    
Leave the first terminal(s) open, and open a new second terminal. In the second terminal, type:  
```
xhost +local:docker
```
This allows the Docker to display content on the screen.  
If this command is successful, in the terminal, you will see:
<img src="https://github.com/guobang494/BlueROV2H-SimTank-ROS-Qualisys/blob/main/INSTALLATION/docker_enable_graphics.png" width=100% height=100%>
  



### 3) Compile the provided simulation code
Return to one of the terminals with the Docker access. This step will compile the code (it takes approx 5 minutes):
```
cd /home/workspaces_ROS/bluerov2h_ws
rm -r devel/ build/ build_isolated/ devel_isolated/ install_isolated/
```
This step might throw a warning such as `rm: cannot remove ...`: not a problem, you can proceed.  
```
catkin_make
source devel/setup.bash
```
Should the `catkin_make` command fail, run it again.  

Add your sourcing to the bashrc file: 
```
echo 'source /home/workspaces_ROS/bluerov2h_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


### 4)  Launch the demo
Open new terminals:
```
sudo docker exec -it bluerov2h_container bash
```
Repeat the last command in 10 terminals (they will be useful later on).   
  
Launch the simulation and control packages:   
```
roslaunch bluerov2_dobmpc start_tank_no_control.launch
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
roslaunch simulation_to_real_bridge simulation_to_real_bridge.launch
roslaunch guidance_law guidance_law.launch
```

You will now see the BlueROV2H vehicle as moving in a path such as in the following example:  
<img src="https://github.com/guobang494/BlueROV2H-SimTank-ROS-Qualisys/blob/main/INSTALLATION/Gazebo_animation.gif" width=100% height=100%>



### 5) Simulation in the future
Open a new terminal, and run: 
```
	xhost +local:docker
```

Open a new terminal, and run:
```
sudo docker start -ai bluerov2h_container
```
Open 10 new terminals (they will be useful later on), and in each of them type:    
```
sudo docker exec -it bluerov2h_container bash
```
  
Launch the simulation and control packages:   
```
roslaunch bluerov2_dobmpc start_tank_no_control.launch
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
roslaunch simulation_to_real_bridge simulation_to_real_bridge.launch
roslaunch guidance_law guidance_law.launch
```


Should you need a larger (virtual) tank, you can for instance launch: 
```
roslaunch bluerov2_dobmpc start_tank_no_control.launch
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
roslaunch simulation_to_real_bridge simulation_to_real_bridge.launch
roslaunch bluerov2_dobmpc start_large_tank_no_control.launch
```


### 6) Edit the Guidance and Control parameters
Instructions are provided in the ![CONTROL_INSTRUCTIONS](./CONTROL_INSTRUCTIONS.md/) file.   


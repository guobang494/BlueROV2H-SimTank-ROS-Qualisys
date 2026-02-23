# Simulation demo instructions
These instructions start a Gazebo demo to run the BlueROV2H controlled via MPC, simulated in the UCL Ocean Towing Tank.   
If you want to change the path scenario, we provide further instructions in the ![Path scenario instructions](./Path_scenario_instructions.md/) file.  

### 0) Pre-requisite
These instructions assume that you have installed the provided Docker container, as explained in ![INSTALLATION](./INSTALLATION.md/) file.     



### 1) Start the docker container
Open a new terminal, and type:
```
sudo docker start -ai my_bluerov_container
```

Note: if you are following up directly from the INSTALLATION instruction file, your container might be already open.   
In that case, run instead:
```
sudo docker exec -it my_bluerov_container bash
```



### 2) Compile the provided simulation code
This step will compile the code (it takes approx 5 minutes):
```
       cd /root/catkin_ws/src/Bluerov2-Simulation-with-docker-env/code/simulation/src/bluerov2/bluerov2_dobmpc/scripts
       python3 generate_c_code.py
       cd /root/catkin_ws
       catkin_make
```
  

### 3) Set gazebo display (not in the Docker)    
Leave the first terminal open, and open a new second terminal. In the second terminal, type:  
```
	xhost +local:docker
```
This allows the Docker to display content on the screen.  
For example, upon launching the demo, you will be able to see:    
<img src="https://github.com/guobang494/Bluerov2-Simulation-with-docker-env/blob/main/INSTALLATION/docker_enable_graphics.png" width=100% height=100%>
  


### 6) Launch the demo
Return to the first terminal with the Docker.
```
	roslaunch /root/catkin_ws/src/Bluerov2-Simulation-with-docker-env/code/simulation/src/bluerov2/bluerov2_dobmpc/launch/start_dobmpc_tank.launch
```


You will now see the BlueROV2H vehicle as moving in a path such as in the following example:  
<img src="https://github.com/guobang494/Bluerov2-Simulation-with-docker-env/blob/main/INSTALLATION/Gazebo_animation.gif" width=100% height=100%>



### 7) Simulation in the future
Open a new terminal, and type:
```
sudo docker start -ai my_bluerov_container
```

Note: if you are following up directly from the INSTALLATION instruction file, your container might be already open.   
In that case, run instead:
```
sudo docker exec -it my_bluerov_container bash
```

```
roslaunch bluerov2_dobmpc start_tank_no_control.launc
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
roslaunch simulation_to_real_bridge simulation_to_real_bridge.launch
```


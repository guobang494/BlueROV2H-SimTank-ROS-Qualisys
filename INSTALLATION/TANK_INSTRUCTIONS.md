# Tank startup instructions
These instructions allow to set up the physical robot in a tank, connecting with the Qualisys camera system.   

### 0) Pre-requisite
These instructions assume that you installed the container, as explained in ![INSTALLATION](./INSTALLATION.md/) file.     


### 1) Start the docker container
Open a new terminal, and type:
```
sudo docker start -ai my_bluerov_container
```

Open 15 new terminals, and type:
```
sudo docker exec -it my_bluerov_container bash
```



### 2) Compile the files
Open one of the terminals and run:   
```
cd ~/catkin_ws/Bluerov2-Simulation-with-docker-env/code/tank-setup
catkin_make 
source devel/setup.bash
```


### 3) Edit the file of the parameters
In the same terminal, locate and open the parameter file:  
```
cd ~/catkin_ws/Bluerov2-Simulation-with-docker-env/code/tank-setup/src/bluerov2_motion_control/configs/
gedit bluerov2_motion_control_config.yaml  
```
In this file, tune your PID gains, the saturation values, and the main motion control parameter.   
To run the motion control, set:   
```
enable_motion_control: true
```

### 4) Launch the motion control
CAVEAT: the next command will launch the motion control! Make sure your parameters make sense before starting.    
  
In a terminal start the BlueROV2H control:  
```
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
```


### 5) Test correct launch of the bluerov2_motion_control package
A dedicated test file is provided in the ![unit test](./unit_testing/bluerov2_motion_control_unit_test.md/) file.     




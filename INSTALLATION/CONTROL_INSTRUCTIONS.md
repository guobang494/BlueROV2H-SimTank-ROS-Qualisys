
### 1) Edit the file of the parameters
In a terminal, locate and open the low-level control parameter file:  
```
cd /home/workspaces_ROS/bluerov2h_ws/src/BlueROV2H-SimTank-ROS-Qualisys/code/guidance_and_control/bluerov2_motion_control/configs/
gedit bluerov2_motion_control_config.yaml  
```
In this file, tune your PID gains, the saturation values, and the main motion control parameter.   
To enable the motion control, set:   
```
enable_motion_control: true
```

Edit the waypoint file:
```
cd /home/workspaces_ROS/bluerov2h_ws/src/BlueROV2H-SimTank-ROS-Qualisys/code/guidance_and_control/guidance_law/configs/
gedit guidance_params.yaml
```
In this file, set the target waypoints.



### 3) Launch the motion control
CAVEAT: the next command will launch the motion control! Make sure your parameters make sense before starting.    
  
In a terminal start the BlueROV2H control:  
```
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
```


### 5) Test correct launch of the bluerov2_motion_control package
A dedicated test file is provided in the ![unit test](./unit_testing/bluerov2_motion_control_unit_test.md/) file.     


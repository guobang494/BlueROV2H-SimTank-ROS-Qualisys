# BlueROV2-Simulation-ROS-Qualisys
This repository contains the code to:  
  
a) **simulate a BlueROV2 Heavy** vehicle in a **Gazebo** environment resembling the University College London Ocean Towing tank;  
b) **control the BlueROV2 Heavy** in a laboratory environment, based on a **ROS (1) middleware**, and using the **Qualisys** motion capture tracking system.  


## Repository structure
The repository contains two main sources of code: 
* the simulation code, located within the ![simulation](./code/simulation/) folder;    
* the code for the setup in a tank environment, located within the ![tank-setup](./code/tank-setup/) folder.
* the code for the ![guidance and control](./code/guidance_and_control) file.   

## Installation 
This software can be run in a **Docker container**, which we provide.
Detailed instructions on installation of the container and on environmental set-up are available within the ![INSTALLATION](./INSTALLATION/INSTALLATION.md/) file.    

In a nutshell, this software employs:  
* Python 3.8
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [uuv simulator](https://uuvsimulator.github.io/)
* [BlueROV2 simulator]()
* [ros-qualisys](https://gepgitlab.laas.fr/gepetto/ros-qualisys)
* [MavROS]()
* [Acados](https://docs.acados.org/installation/index.html)

This repository is provided as a standalone software architecture, ensuring compatibility across all the listed modules (and more).  


## Simulation 
Upon following the installation instructions, you will be able to run a control architecture composed of a Model Predictive Control in the virtual water tank.   
You will see the BlueROV2H vehicle as:  
<img src="https://github.com/guobang494/Bluerov2-Simulation-with-docker-env/blob/main/INSTALLATION/BlueROV2H_UCL_Ocean_Towing_Tank.png" width=100% height=100%>
  moving in a path such as in the following example:  
<img src="https://github.com/guobang494/Bluerov2-Simulation-with-docker-env/blob/main/INSTALLATION/Gazebo_animation.gif" width=100% height=100%>

The full list of instructions to reproduce the demo are available within the ![SIMULATION_INSTRUCTIONS](./INSTALLATION/SIMULATION_INSTRUCTIONS.md/) file.    




  




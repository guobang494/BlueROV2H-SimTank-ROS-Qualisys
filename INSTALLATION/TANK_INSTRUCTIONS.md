# Tank startup instructions
These instructions allow to set up the physical robot in a tank, connecting with the Qualisys camera system.   
Steps 0 to 8 will only be needed the first time you set up the system.  

This installation requires: 
* a computer running Linux (the one used to follow the previous ![INSTALLATION.md](./INSTALLATION.md) instructions file. This is referred to as **topside computer**.
* a Qualisys system (either in air, or underwater, or both)
* a computer running Windows with the Qualisys QTM installed 
* a Wi-Fi router to connect the two computers (an ethernet connection might be used instead, but we tested with the Wi-Fi router only) 
* a BlueROV2H connected to the Linux computer

### 0) Pre-requisite
These instructions assume that you have already completed the Docker and repository setup from the ![INSTALLATION.md](./INSTALLATION.md) file, including the separation of the Qualisys workspace into:
- `/home/workspaces_ROS/bluerov2h_ws`
- `/home/workspaces_ROS/ros_qualisys_ws`


### 1) Start the docker container
Open a new terminal, and type:
```
sudo docker start -ai bluerov2h_container
```

Open 10 new terminals, and type:
```
sudo docker exec -it bluerov2h_container bash
```



### 2) Compile the BlueRov2H workspace
This step will compile the code (it takes approx 5 minutes):
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


### 3) Install colcon-related dependencies
Run
```
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-noetic-roscpp \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-geometry-msgs \
  libboost-all-dev \
  cmake \
  git \
  doxygen
```



### 4) Adjust and build the Qualisys sdk
Ensuring the correct version of CMake is used for this compilation: 
```
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
hash -r
which cmake
cmake --version
```
This should return: `cmake version 3.16.3`  


Recompile the qualisys_cpp_sdk package (this step takes ~5 min):
```
cd /home/workspaces_ROS/bluerov2h_ws/src/BlueROV2H-SimTank-ROS-Qualisys/code/tank-setup/src/qualisys_cpp_sdk

rm -rf build
rm -rf /home/workspaces_ROS/qualisys_cpp_sdk_install

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dqualisys_cpp_sdk_OUTPUT_TYPE=SHARED

cmake --build build --config Release
cmake --install build --prefix /home/workspaces_ROS/qualisys_cpp_sdk_install --config Release

mkdir -p /home/workspaces_ROS/qualisys_cpp_sdk_install/include/qualisys_cpp_sdk

cp /home/workspaces_ROS/bluerov2h_ws/src/BlueROV2H-SimTank-ROS-Qualisys/code/tank-setup/src/qualisys_cpp_sdk/*.h \
   /home/workspaces_ROS/qualisys_cpp_sdk_install/include/qualisys_cpp_sdk/
 ```
 
 Add environment definition to the .bashrc file:
 ```
 cat <<'EOF' >> ~/.bashrc

# ROS Noetic
source /opt/ros/noetic/setup.bash

# Qualisys workspace
if [ -f /home/workspaces_ROS/ros_qualisys_ws/install/setup.bash ]; then
  source /home/workspaces_ROS/ros_qualisys_ws/install/setup.bash
fi

# Qualisys SDK
export CMAKE_PREFIX_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install:$CMAKE_PREFIX_PATH
export qualisys_cpp_sdk_DIR=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib/qualisys_cpp_sdk
export LD_LIBRARY_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib:$LD_LIBRARY_PATH
EOF
```
and source it:
```
source ~/.bashrc
```

Now build the second workspace:
```
cd /home/workspaces_ROS/ros_qualisys_ws
rm -rf build install log

colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install \
  -Dqualisys_cpp_sdk_DIR=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib/qualisys_cpp_sdk
```


### 5) Set up the BlueROV2H
Place the Qualisys marker on the BlueROV2H. An example of how we mounted them for surface-only operations is shows below:

<img src="tank_images/bluerov2h_markers.jpg" width="80%">

Switch on the BlueROV2H (plug in the battery).   



### 6) Set up the Qualisys system

Switch on the Qualisys cameras, your Windows computer, and launch 'Qualisys QTM'.   

Place the previously set-up BlueROV2H in the field of view of the cameras, and define a new rigid body called `BlueROV2H` (the naming is important!).  



### 7) Connect the Linux computer to the BlueROV2
Plug in the BlueRobotics Fathom-X Topside Interface to your laptop via USB.  

<img src="tank_images/fathomx.jpg" width="60%">




### 8) Network setup 
Switch on your Wi-Fi router. As an example, we are using the following one: 
<img src="tank_images/router.jpg" width="60%">
   
Configure the **topside computer** with a static IP as follows.  
Go to system settings, Networks, and enable the 'USB Ethernet' option. 
<img src="tank_images/network_1.png" width="60%">

Now press on the gearbox icon on the right hand-side of the 'USB Ethernet' and edit IPv4 as follows:
```
IPv4 Method: Manual
IP Address: 192.168.2.1
Subnet Mask: 255.255.255.0
```

<img src="tank_images/network_2.png" width="60%">

If needed, you can refer to the official BlueROV2 networking/software instructions from BlueRobotics:
https://bluerobotics.com/learn/bluerov2-software-setup-r3-and-older/#software-introduction


Test connection:
```
sudo apt-get install iputils-ping
ping 192.168.2.1
```
You should now see:  
<img src="tank_images/ping.png" width="100%">




### 9) Start the MAVROS connection
In an open terminal, run MAVROS:

```
roslaunch mavros apm.launch \
fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 \
target_system_id:=1 \
target_component_id:=1
```
If successful, MAVROS will start receiving **heartbeat messages**.

<img src="terminal_images/mavros success" width="100%">


Based on your BlueROV2H version, you might also notice a series of warnings and errors, such as the following: 

<img src="tank_images/mavros_errors.png" width="100%">

This is not critical, you can proceed with the set up.


Check MAVROS state:

```
rostopic echo /mavros/state
```
You should see the status as follows:  
<img src="terminal_images/mavros check status" width="50%">


### 10) Launch the MAVROS PWM 
Launch the PWM publisher:
```
roslaunch /home/workspaces_ROS/bluerov2h_ws/src/BlueROV2H-SimTank-ROS-Qualisys/code/guidance_and_control/mavros_pub/launch/pwm_pub.launch
```

### 11) Launch Qualisys ROS pkg
Launch:
```
roslaunch ~/bluerov2_pid/ros_qualysis/src/launch/qualisys_bauzil_bringup.launch
```

If necessary, modify the server IP address in:
```
roslaunch ros_qualysis/src/launch/qualisys_bauzil_bringup.launch server_address:=xxx.xx.xx.x     server_base_port:=xxxxx
```
If successful, you will see: 

<img src="terminal_images/qualisys success" width="50%">



### 10) Launch the guidance and control 
In one terminal:
```
roslaunch bluerov2_motion_control bluerov2_motion_control.launch
```

In another terminal:
```
roslaunch guidance_law guidance_law.launch
```

### 11) Convert Qualisys data into ROS pose

```
python3 /home/workspaces_ROS/ros_qualisys_ws/src/ros_qualysis/scripts/tf2_pose_gt_real.py
```


### 12 Arm / Disarm
CAVEAT: Arming the robot will start the robot!
```
# Arm
rosservice call /mavros/cmd/arming "value: true"
# Disarm
rosservice call /mavros/cmd/arming "value: false"
```


### 13) Edit the Guidance and Control parameters
If needed, further instructions to modify the control system are provided in the ![CONTROL_INSTRUCTIONS](./CONTROL_INSTRUCTIONS.md/) file.   




# BlueROV2 ROS Docker information

This document describes the key steps we took to set up a Docker with the dependencies for a **BlueROV2 using ROS Noetic, MAVROS, Qualisys, and PID control**.

---

# 1. MAVROS Installation 

Install MAVROS and related packages.

```bash
sudo apt-get update

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod a+x install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh
```

Install RQT tools:

```bash
sudo apt-get install \
ros-noetic-rqt \
ros-noetic-rqt-common-plugins \
ros-noetic-rqt-robot-plugins
```

---

# 2. BlueROV2 Network Setup

Configure the **topside computer** with a static IP:

```
IP Address: 192.168.2.1
Subnet Mask: 255.255.255.0
```
You can also see the official instruction for Bluerov2 which the link is locatied in 
https://bluerobotics.com/learn/bluerov2-software-setup-r3-and-older/#software-introduction


# 3. Launch MAVROS

Run MAVROS:

```bash
roslaunch mavros apm.launch \
fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 \
target_system_id:=1 \
target_component_id:=1
```


If successful, MAVROS will start receiving **heartbeat messages**.

<img src="terminal_images/mavros success" width="100%">

Check MAVROS state:

```bash
rostopic echo /mavros/state
```
You can see the status like this 

<img src="terminal_images/mavros check status" width="50%">

---

# 4. Arm and Disarm BlueROV2

Disarm:

```bash
rosservice call /mavros/cmd/arming "value: false"
```

Arm:

```bash
rosservice call /mavros/cmd/arming "value: true"
```

---

# 5. Install Qualisys ROS Package (If you use docker we provided then skip this)

Download the package into:

```
~/bluerov2_pid/ros_qualysis
```

Setup environment:

```bash
cd ~/bluerov2_pid/ros_qualysis

export CMAKE_PREFIX_PATH=/opt/openrobots

source /opt/ros/noetic/setup.bash
```

Build the workspace:

```bash
cd ~/bluerov2_pid/ros_qualysis

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

# 6. Launch Qualisys ROS Node

Load environment:

```bash
source /opt/ros/noetic/setup.bash

export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH

source ./install/setup.bash
```

Launch:

```bash
roslaunch ~/bluerov2_pid/ros_qualysis/src/launch/qualisys_bauzil_bringup.launch
```

If necessary, modify the server IP address in:

```
Roslaunch ros_qualysis/src/launch/qualisys_bauzil_bringup.launch server_address:=xxx.xx.xx.x     server_base_port:=xxxxx
```
If success ,if you will see 

<img src="terminal_images/qualisys success" width="50%">
---

# 7. Transform Qualisys Data to ROS Pose

Run the transformation script:

```bash
python3 ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py
```

This converts **Qualisys motion capture data** into **ROS pose data**.
<img src="terminal_images/data transfer success" width="75%">

---


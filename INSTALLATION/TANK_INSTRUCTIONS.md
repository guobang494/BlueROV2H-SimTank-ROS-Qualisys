# Tank startup instructions
These instructions allow to set up the physical robot in a tank, connecting with the Qualisys camera system.   



### 0) Pre-requisite
These instructions assume that you have installed the container, as explained in ![INSTALLATION](./INSTALLATION.md/) file.     


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
       catkin_make
       source devel/setup.bash
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



### 4) Adjust compilation of the the Qualisys sdk
Recompile the qualisys_cpp_sdk package (this step takes ~5 min):
```
cd /home/workspaces_ROS/bluerov2h_ws/src/Bluerov2-Simulation-with-docker-env/code/tank-setup/src/qualisys_cpp_sdk

rm -rf build
rm -rf /home/workspaces_ROS/qualisys_cpp_sdk_install

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dqualisys_cpp_sdk_OUTPUT_TYPE=SHARED

cmake --build build --config Release
cmake --install build --prefix /home/workspaces_ROS/qualisys_cpp_sdk_install --config Release

mkdir -p /home/workspaces_ROS/qualisys_cpp_sdk_install/include/qualisys_cpp_sdk

cp /home/workspaces_ROS/bluerov2h_ws/src/Bluerov2-Simulation-with-docker-env/code/tank-setup/src/qualisys_cpp_sdk/*.h \
   /home/workspaces_ROS/qualisys_cpp_sdk_install/include/qualisys_cpp_sdk/
   
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
hash -r
source /opt/ros/noetic/setup.bash

export CMAKE_PREFIX_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install:$CMAKE_PREFIX_PATH
export qualisys_cpp_sdk_DIR=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib/qualisys_cpp_sdk
export LD_LIBRARY_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib:$LD_LIBRARY_PATH
   
```

Compile the second workspace:
```

cd /home/workspaces_ROS/ros_qualisys_ws
rm -rf build install log

colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/workspaces_ROS/qualisys_cpp_sdk_install \
  -Dqualisys_cpp_sdk_DIR=/home/workspaces_ROS/qualisys_cpp_sdk_install/lib/qualisys_cpp_sdk
```



### 6) Edit the Guidance and Control parameters
Instructions are provided in the ![INSTALLATION](./CONTROL_INSTRUCTIONS.md/) file.   



# Installation instructions

The installation provided below is composed of 6 steps.   
Overall, you should be able to install the dependencies, and test successful deployment of the code in simulation within 30 minutes.   


### 1) Install Docker
If you do not have **Docker** installed on your machine, please find the complete set instructions available here:   
[Docker installation](https://docs.docker.com/engine/install/ubuntu/)

If you are on Linux Ubuntu, you can install it by running:  
```
sudo apt install gnome-terminal
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Confirm that your Docker installation is successful: 
```
sudo docker run hello-world
```
This will print:  
<img src="https://github.com/guobang494/Bluerov2-Simulation-with-docker-env/blob/main/INSTALLATION/docker_hello_world_terminal.png" width=100% height=100%>
  
### 2) Download the code
Clone the repo into the standards catkin workspace:  
```
mkdir -p  ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/guobang494/Bluerov2-Simulation-with-docker-env
```
   
### 3) Install Docker Image
Install the Docker image that we provide.   
This step requires approx. 10 minutes, and 3Gb of space on your hard-drive.  
```
      sudo docker pull zebangg/bluerov2_package:v1.0
      sudo docker run -it \
          --name bluerov2h_container \
          --network host \
          --privileged \
          -v /tmp/.X11-unix:/tmp/.X11-unix \
          -v ~/catkin_ws:/root/catkin_ws \
          -e DISPLAY=$DISPLAY \
          zebangg/bluerov2_package:v1.0 \
          bash
  ```
  
  You are now in the Docker container.  Verify folder successful created by running: 
  ```
  cd ~/catkin_ws/src
  ```
  You should see the folder previously downloaded: ```Bluerov2-Simulation-with-docker-env```
  
  
### 4) Sourcing the code 
Open your bashrc and permanently source the catkin workspace. Use ```gedit ~/.bashrc)``` and add the following line to the file:  
```
source /root/catkin_ws/devel/setup.bash
```
  

### 5) Next steps
You can now follow up with the instructions to run the code in:
1) ![a simulation environment](./SIMULATION_INSTRUCTIONS.md/)
2) ![on a real vehicle](./TANK_INSTRUCTIONS.md/)
 














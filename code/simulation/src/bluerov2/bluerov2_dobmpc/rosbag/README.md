# BlueROV2 DOB-MPC Data Recording

This directory contains rosbag scripts used to record runtime data for the BlueROV2 DOB-MPC system.

## Usage

### 1. Start the simulation
First launch the BlueROV2 DOB-MPC simulation:
```bash
roslaunch bluerov2_dobmpc start_dobmpc_demo.launch
```

### 2. Start data recording
Run the recording script in a new terminal:
```bash
cd /home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag
./record
```

Or run it from any location:
```bash
/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/record
```

### 3. Stop recording
Press `Ctrl+C` to stop recording.

## Recorded data

The script records the following ROS topics:

### Position and reference information
- `/bluerov2/pose_gt` - Ground-truth robot position and velocity (nav_msgs/Odometry)
- `/bluerov2/mpc/reference` - MPC reference trajectory (nav_msgs/Odometry)
- `/bluerov2/mpc/error` - MPC tracking error (nav_msgs/Odometry)
- `/bluerov2/ekf/pose` - EKF estimated pose (nav_msgs/Odometry)
- `/bluerov2/ekf/disturbance` - Estimated disturbance (nav_msgs/Odometry)

### Thruster data (8 thrusters)
- `/bluerov2/thrusters/0/thrust` to `/bluerov2/thrusters/7/thrust` - Actual thruster thrust
- `/bluerov2/thrusters/0/input` to `/bluerov2/thrusters/7/input` - Thruster input commands

### Control inputs
- `/bluerov2/control_input/0` to `/bluerov2/control_input/3` - Controller inputs

## Output file

The rosbag file is automatically saved in the current directory with the following name format:
`bluerov2_dobmpc_YYYYMMDD_HHMMSS.bag`

For example: `bluerov2_dobmpc_20231215_143022.bag`

## Data analysis

Recorded data can be used for:
- Control performance analysis
- Trajectory tracking evaluation
- Thruster efficiency studies
- System identification
- Disturbance observer performance evaluation

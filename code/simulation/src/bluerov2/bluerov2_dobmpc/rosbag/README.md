# BlueROV2 DOB-MPC 数据记录

这个目录包含用于记录BlueROV2 DOB-MPC系统运行数据的rosbag脚本。

## 使用方法

### 1. 启动仿真系统
首先启动BlueROV2 DOB-MPC仿真：
```bash
roslaunch bluerov2_dobmpc start_dobmpc_demo.launch
```

### 2. 开始数据记录
在新的终端中运行记录脚本：
```bash
cd /home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag
./record
```

或者从任何位置运行：
```bash
/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/record
```

### 3. 停止记录
按 `Ctrl+C` 停止记录

## 记录的数据

脚本会记录以下ROS话题：

### 位置和参考信息
- `/bluerov2/pose_gt` - 机器人真实位置和速度 (nav_msgs/Odometry)
- `/bluerov2/mpc/reference` - MPC参考轨迹 (nav_msgs/Odometry)
- `/bluerov2/mpc/error` - MPC跟踪误差 (nav_msgs/Odometry)
- `/bluerov2/ekf/pose` - EKF估计位置 (nav_msgs/Odometry)
- `/bluerov2/ekf/disturbance` - 估计扰动 (nav_msgs/Odometry)

### 推进器数据（8个推进器）
- `/bluerov2/thrusters/0/thrust` 到 `/bluerov2/thrusters/7/thrust` - 推进器实际推力
- `/bluerov2/thrusters/0/input` 到 `/bluerov2/thrusters/7/input` - 推进器输入指令

### 控制输入
- `/bluerov2/control_input/0` 到 `/bluerov2/control_input/3` - 控制系统输入

## 输出文件

rosbag文件会自动保存在当前目录下，文件名格式为：
`bluerov2_dobmpc_YYYYMMDD_HHMMSS.bag`

例如：`bluerov2_dobmpc_20231215_143022.bag`

## 数据分析

记录的数据可以用于：
- 控制性能分析
- 轨迹跟踪评估  
- 推进器效率研究
- 系统识别
- 扰动观测器性能评估 
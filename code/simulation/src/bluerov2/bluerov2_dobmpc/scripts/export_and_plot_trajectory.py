#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import signal
import sys
import numpy as np
import os

trajectory = []

# 参考轨迹文件路径（可根据实际情况修改）
REF_TRAJ_PATH = os.path.expanduser("/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_mpc/traj/lemniscate.txt")

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    trajectory.append((x, y, z))

def save_and_plot():
    # 保存为txt
    with open("trajectory_xyz.txt", "w") as f:
        for x, y, z in trajectory:
            f.write(f"{x} {y} {z}\n")
    print("轨迹已保存为 trajectory_xyz.txt")

    # 读取参考轨迹
    if os.path.exists(REF_TRAJ_PATH):
        ref = np.loadtxt(REF_TRAJ_PATH)
        x_ref, y_ref, z_ref = ref[:,0], ref[:,1], ref[:,2]
        print("已加载参考轨迹。")
    else:
        x_ref, y_ref, z_ref = [], [], []
        print("未找到参考轨迹文件，图中不显示参考轨迹。")

    # 绘图
    if len(trajectory) > 1:
        xs, ys, zs = zip(*trajectory)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xs, ys, zs, label='实际轨迹', color='b')
        if len(x_ref) > 0:
            ax.plot(x_ref, y_ref, z_ref, 'g--', label='参考轨迹')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title('BlueROV2 三维轨迹对比')
        ax.legend()
        
        # 设置合适的Z轴范围以便更好地显示Z方向的变化
        all_z = list(zs)
        if len(z_ref) > 0:
            all_z.extend(z_ref)
        
        if len(all_z) > 0:
            z_min, z_max = min(all_z), max(all_z)
            z_range = z_max - z_min
            
            # 如果Z方向变化很小，设置一个合适的显示范围
            if z_range < 1.0:  # 如果Z方向变化小于1米
                z_center = (z_min + z_max) / 2
                z_margin = max(0.5, z_range * 2)  # 至少显示1米范围，或者是实际范围的2倍
                ax.set_zlim(z_center - z_margin, z_center + z_margin)
                print(f"设置Z轴范围: {z_center - z_margin:.2f} 到 {z_center + z_margin:.2f}")
            else:
                # 如果Z方向变化较大，使用默认的边距
                z_margin = z_range * 0.1
                ax.set_zlim(z_min - z_margin, z_max + z_margin)
        
        plt.show()
    else:
        print("没有足够的轨迹点绘图。")

def signal_handler(sig, frame):
    print("\n检测到退出，正在保存并绘图...")
    save_and_plot()
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node("export_and_plot_trajectory")
    rospy.Subscriber("/bluerov2/pose_gt", Odometry, pose_callback)
    print("正在记录轨迹，按 Ctrl+C 结束并自动保存与绘图...")
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin() 

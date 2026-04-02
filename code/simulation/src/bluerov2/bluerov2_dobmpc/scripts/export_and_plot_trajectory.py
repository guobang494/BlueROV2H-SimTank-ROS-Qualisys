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

# Reference trajectory file path (adjust as needed)
REF_TRAJ_PATH = os.path.expanduser("/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_mpc/traj/lemniscate.txt")

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    trajectory.append((x, y, z))

def save_and_plot():
    # Save to txt
    with open("trajectory_xyz.txt", "w") as f:
        for x, y, z in trajectory:
            f.write(f"{x} {y} {z}\n")
    print("Trajectory saved to trajectory_xyz.txt")

    # Load reference trajectory
    if os.path.exists(REF_TRAJ_PATH):
        ref = np.loadtxt(REF_TRAJ_PATH)
        x_ref, y_ref, z_ref = ref[:,0], ref[:,1], ref[:,2]
        print("Reference trajectory loaded.")
    else:
        x_ref, y_ref, z_ref = [], [], []
        print("Reference trajectory file not found; it will not be shown in the plot.")

    # Plot
    if len(trajectory) > 1:
        xs, ys, zs = zip(*trajectory)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xs, ys, zs, label='Actual trajectory', color='b')
        if len(x_ref) > 0:
            ax.plot(x_ref, y_ref, z_ref, 'g--', label='Reference trajectory')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title('BlueROV2 3D Trajectory Comparison')
        ax.legend()
        
        # Set a suitable Z-axis range to better visualize Z-direction changes
        all_z = list(zs)
        if len(z_ref) > 0:
            all_z.extend(z_ref)
        
        if len(all_z) > 0:
            z_min, z_max = min(all_z), max(all_z)
            z_range = z_max - z_min
            
            # If Z-direction changes are small, set an explicit range
            if z_range < 1.0:  # If Z variation is smaller than 1 meter
                z_center = (z_min + z_max) / 2
                z_margin = max(0.5, z_range * 2)  # Show at least 1 m range, or 2x actual range
                ax.set_zlim(z_center - z_margin, z_center + z_margin)
                print(f"Set Z-axis range: {z_center - z_margin:.2f} to {z_center + z_margin:.2f}")
            else:
                # If Z-direction changes are larger, use default margins
                z_margin = z_range * 0.1
                ax.set_zlim(z_min - z_margin, z_max + z_margin)
        
        plt.show()
    else:
        print("Not enough trajectory points to plot.")

def signal_handler(sig, frame):
    print("\nExit detected, saving data and plotting...")
    save_and_plot()
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node("export_and_plot_trajectory")
    rospy.Subscriber("/bluerov2/pose_gt", Odometry, pose_callback)
    print("Recording trajectory. Press Ctrl+C to stop, then auto-save and plot.")
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin() 

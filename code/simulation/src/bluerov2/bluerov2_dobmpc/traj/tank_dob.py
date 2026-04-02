#--------------------------------------
# Generate Complete Figure-8 reference trajectory for NMPC in Tank Environment
# Tank dimensions: 18.15m x 2.5m x 1m
# Trajectory boundaries check (slender full figure-8):
# - x range: [-6.5, +6.5] (within tank x: [-9.075, +9.075]) - 13.0 m span
# - y range: [-0.45, +0.45] (within tank y: [-1.25, +1.25]) - 0.9 m span
# - z: -0.5 (within tank z: [0, -1])
# - start/end position: (0, 0, -0.5) - starts and ends at tank center
# - aspect ratio: 28.9:1 (very slender figure-8)
#--------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parameters
sample_time = 0.05   #
duration = 180;                      #seconds

# Full figure-8 trajectory parameters (fixed start position and return)
amp_x = 6.5    # x amplitude (long side of figure-8)
amp_y = 0.45    # y amplitude (short side of figure-8)
frq = 2*np.pi/180 # Frequency, one full figure-8 in 60 seconds

# Fixed initial position
x0 = 0    # Initial X position: 5.575 m from left tank end, 14.65 m from right end
y0 = 0       # Initial Y position: tank center
z0 = -0.5    # Initial Z position: mid-depth

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

# Full figure-8 trajectory equations (returns to start point)
# Standard figure-8 parametric form: x = a*sin(t), y = b*sin(2t)
# At t=0 and t=2*pi, the path returns to origin, creating a closed figure-8
traj[:,0] = amp_x*np.sin(t*frq) + x0           # x - figure-8 lateral motion
traj[:,1] = amp_y*np.sin(2*t*frq) + y0         # y - figure-8 longitudinal motion (double frequency crossing)
traj[:,2] = z0                                 # z - fixed depth
traj[:,3] = 0                                  # phi (roll)
traj[:,4] = 0                                  # theta (pitch)
traj[:,5] = 0                                  # psi (yaw)

# Corresponding velocities (derivatives)
traj[:,6] = amp_x*frq*np.cos(t*frq)            # u (x velocity)
traj[:,7] = amp_y*2*frq*np.cos(2*t*frq)        # v (y velocity)
traj[:,8] = 0                                  # w (z velocity)
traj[:,9] = 0                       # p
traj[:,10] = 0                      # q
traj[:,11] = 0                      # r
traj[:,12] = 0                      # u1
traj[:,13] = 0                      # u2
traj[:,14] = 0                      # u2
traj[:,15] = 0                      # u2

# Add extra points for MPC horizon (simple fix)
last_point = traj[-1,:]  # Get the last point
extra_points = np.tile(last_point, (20, 1))  # Repeat last point 20 times
traj = np.vstack([traj, extra_points])  # Append to trajectory

# write to txt
np.savetxt('tank_dob.txt',traj,fmt='%f')
print(f"Full figure-8 trajectory generated: tank_dob.txt")
print(f"Number of trajectory points: {traj.shape[0]}")
print(f"x range: [{np.min(traj[:,0]):.3f}, {np.max(traj[:,0]):.3f}]")
print(f"y range: [{np.min(traj[:,1]):.3f}, {np.max(traj[:,1]):.3f}]")
print(f"z depth: {traj[0,2]}")

# # Visualize trajectory
# plt.figure(figsize=(15, 10))

# # Tank boundary definitions
# tank_x_min, tank_x_max = -9.075, 9.075  # 18.15 m length
# tank_y_min, tank_y_max = -1.25, 1.25    # 2.5 m width
# tank_z_min, tank_z_max = -1.0, 0.0      # 1 m depth

# # 3D view - full trajectory
# ax1 = plt.subplot(2, 2, 1, projection='3d')
# ax1.plot(traj[:,0], traj[:,1], traj[:,2], 'b-', linewidth=2, label='Full figure-8 trajectory')
# ax1.scatter(traj[0,0], traj[0,1], traj[0,2], color='green', s=100, label=f'Start ({traj[0,0]:.1f}, {traj[0,1]:.1f})')
# ax1.scatter(traj[-21,0], traj[-21,1], traj[-21,2], color='red', s=100, label=f'End ({traj[-21,0]:.1f}, {traj[-21,1]:.1f})')
# # Mark the figure-8 center point
# center_idx = len(traj)//4
# ax1.scatter(traj[center_idx,0], traj[center_idx,1], traj[center_idx,2], color='orange', s=100, label='Figure-8 center')

# # Draw tank boundaries
# # Bottom surface
# xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
# zz = np.full_like(xx, tank_z_min)
# ax1.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)') 
# ax1.set_zlabel('Z (m)')
# ax1.set_title('Full Figure-8 Trajectory in 3D Tank')
# ax1.legend()
# ax1.set_xlim(tank_x_min, tank_x_max)
# ax1.set_ylim(tank_y_min, tank_y_max)
# ax1.set_zlim(tank_z_min, tank_z_max)

# # XY plane view
# ax2 = plt.subplot(2, 2, 2)
# ax2.plot(traj[:,0], traj[:,1], 'b-', linewidth=2, label='Full figure-8 trajectory')
# ax2.scatter(traj[0,0], traj[0,1], color='green', s=100, label=f'Start/End ({traj[0,0]:.1f}, {traj[0,1]:.1f})')
# ax2.scatter(traj[center_idx,0], traj[center_idx,1], color='orange', s=100, label='Figure-8 center')

# # Tank boundary
# ax2.add_patch(plt.Rectangle((tank_x_min, tank_y_min), 
#                            tank_x_max-tank_x_min, tank_y_max-tank_y_min, 
#                            fill=False, edgecolor='gray', linewidth=2, label='Tank boundary'))
# ax2.set_xlabel('X (m)')
# ax2.set_ylabel('Y (m)')
# ax2.set_title('XY Plane View (Top View)')
# ax2.legend()
# ax2.grid(True)
# ax2.axis('equal')
# ax2.set_xlim(tank_x_min-1, tank_x_max+1)
# ax2.set_ylim(tank_y_min-1, tank_y_max+1)

# # X-time plot
# ax3 = plt.subplot(2, 2, 3)
# time = np.arange(len(traj[:int(duration/sample_time+1)])) * sample_time
# ax3.plot(time, traj[:len(time),0], 'r-', linewidth=2, label='X position')
# ax3.axhline(y=tank_x_min, color='gray', linestyle='--', label='Tank X boundary')
# ax3.axhline(y=tank_x_max, color='gray', linestyle='--')
# ax3.set_xlabel('Time (s)')
# ax3.set_ylabel('X position (m)')
# ax3.set_title('X Position vs Time')
# ax3.legend()
# ax3.grid(True)

# # Y-time plot
# ax4 = plt.subplot(2, 2, 4)
# ax4.plot(time, traj[:len(time),1], 'g-', linewidth=2, label='Y position')
# ax4.axhline(y=tank_y_min, color='gray', linestyle='--', label='Tank Y boundary')
# ax4.axhline(y=tank_y_max, color='gray', linestyle='--')
# ax4.set_xlabel('Time (s)')
# ax4.set_ylabel('Y position (m)')
# ax4.set_title('Y Position vs Time')
# ax4.legend()
# ax4.grid(True)

# plt.tight_layout()
# plt.savefig('tank_dob.png', dpi=300, bbox_inches='tight')
# plt.show()
# print(f"Trajectory plot saved: tank_dob.png")

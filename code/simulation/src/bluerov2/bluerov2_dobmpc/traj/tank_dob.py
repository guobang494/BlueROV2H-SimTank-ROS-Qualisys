#--------------------------------------
# Generate Complete Figure-8 reference trajectory for NMPC in Tank Environment
# Tank dimensions: 18.15m x 2.5m x 1m
# Trajectory boundaries check (细长完整八字形):
# - x range: [-6.5, +6.5] (within tank x: [-9.075, +9.075]) - 13.0m跨度
# - y range: [-0.45, +0.45] (within tank y: [-1.25, +1.25]) - 0.9m跨度
# - z: -0.5 (within tank z: [0, -1])
# - 初始/结束位置: (0, 0, -0.5) - tank中央开始和结束
# - 长宽比: 28.9:1 (非常细长的八字形)
#--------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parameters
sample_time = 0.05   #
duration = 180;                      #seconds

# 完整八字形轨迹参数 (确定初始位置和回归)
amp_x = 6.5    # x方向幅度 (八字形长边)
amp_y = 0.45    # y方向幅度 (八字形短边)
frq = 2*np.pi/180 # 频率，60秒完成一个完整八字

# 确定的初始位置
x0 = 0    # 初始X位置：距tank左端5.575m，距右端14.65m
y0 = 0       # 初始Y位置：tank中央
z0 = -0.5    # 初始Z位置：tank中间深度

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

# 完整八字形轨迹方程 (Figure-8, 确保回到起始点)
# 标准八字形参数方程：x = a*sin(t), y = b*sin(2t)
# 在t=0和t=2π时都回到原点，形成完整闭合八字
traj[:,0] = amp_x*np.sin(t*frq) + x0           # x - 八字形横向运动
traj[:,1] = amp_y*np.sin(2*t*frq) + y0         # y - 八字形纵向运动（双频率形成交叉）
traj[:,2] = z0                                 # z - 固定深度
traj[:,3] = 0                                  # phi (roll)
traj[:,4] = 0                                  # theta (pitch)
traj[:,5] = 0                                  # psi (yaw)

# 对应的速度 (derivatives)
traj[:,6] = amp_x*frq*np.cos(t*frq)            # u (x方向速度)  
traj[:,7] = amp_y*2*frq*np.cos(2*t*frq)        # v (y方向速度)
traj[:,8] = 0                                  # w (z方向速度)
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
print(f"完整八字形轨迹已生成: tank_dob.txt")
print(f"轨迹点数: {traj.shape[0]}")
print(f"x范围: [{np.min(traj[:,0]):.3f}, {np.max(traj[:,0]):.3f}]")
print(f"y范围: [{np.min(traj[:,1]):.3f}, {np.max(traj[:,1]):.3f}]")
print(f"z深度: {traj[0,2]}")

# # 可视化轨迹
# plt.figure(figsize=(15, 10))

# # Tank边界定义
# tank_x_min, tank_x_max = -9.075, 9.075  # 18.15m长度
# tank_y_min, tank_y_max = -1.25, 1.25    # 2.5m宽度  
# tank_z_min, tank_z_max = -1.0, 0.0      # 1m深度

# # 3D图 - 整体轨迹
# ax1 = plt.subplot(2, 2, 1, projection='3d')
# ax1.plot(traj[:,0], traj[:,1], traj[:,2], 'b-', linewidth=2, label='完整八字形轨迹')
# ax1.scatter(traj[0,0], traj[0,1], traj[0,2], color='green', s=100, label=f'起点({traj[0,0]:.1f}, {traj[0,1]:.1f})')
# ax1.scatter(traj[-21,0], traj[-21,1], traj[-21,2], color='red', s=100, label=f'终点({traj[-21,0]:.1f}, {traj[-21,1]:.1f})')
# # 标记八字中心点
# center_idx = len(traj)//4
# ax1.scatter(traj[center_idx,0], traj[center_idx,1], traj[center_idx,2], color='orange', s=100, label='八字中心')

# # 绘制tank边界
# # 底面
# xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
# zz = np.full_like(xx, tank_z_min)
# ax1.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)') 
# ax1.set_zlabel('Z (m)')
# ax1.set_title('3D Tank中的完整八字形轨迹')
# ax1.legend()
# ax1.set_xlim(tank_x_min, tank_x_max)
# ax1.set_ylim(tank_y_min, tank_y_max)
# ax1.set_zlim(tank_z_min, tank_z_max)

# # XY平面视图
# ax2 = plt.subplot(2, 2, 2)
# ax2.plot(traj[:,0], traj[:,1], 'b-', linewidth=2, label='完整八字形轨迹')
# ax2.scatter(traj[0,0], traj[0,1], color='green', s=100, label=f'起点/终点({traj[0,0]:.1f}, {traj[0,1]:.1f})')
# ax2.scatter(traj[center_idx,0], traj[center_idx,1], color='orange', s=100, label='八字中心')

# # Tank边界
# ax2.add_patch(plt.Rectangle((tank_x_min, tank_y_min), 
#                            tank_x_max-tank_x_min, tank_y_max-tank_y_min, 
#                            fill=False, edgecolor='gray', linewidth=2, label='Tank边界'))
# ax2.set_xlabel('X (m)')
# ax2.set_ylabel('Y (m)')
# ax2.set_title('XY平面视图 (俯视图)')
# ax2.legend()
# ax2.grid(True)
# ax2.axis('equal')
# ax2.set_xlim(tank_x_min-1, tank_x_max+1)
# ax2.set_ylim(tank_y_min-1, tank_y_max+1)

# # X-时间图
# ax3 = plt.subplot(2, 2, 3)
# time = np.arange(len(traj[:int(duration/sample_time+1)])) * sample_time
# ax3.plot(time, traj[:len(time),0], 'r-', linewidth=2, label='X位置')
# ax3.axhline(y=tank_x_min, color='gray', linestyle='--', label='Tank X边界')
# ax3.axhline(y=tank_x_max, color='gray', linestyle='--')
# ax3.set_xlabel('时间 (s)')
# ax3.set_ylabel('X 位置 (m)')
# ax3.set_title('X位置随时间变化')
# ax3.legend()
# ax3.grid(True)

# # Y-时间图  
# ax4 = plt.subplot(2, 2, 4)
# ax4.plot(time, traj[:len(time),1], 'g-', linewidth=2, label='Y位置')
# ax4.axhline(y=tank_y_min, color='gray', linestyle='--', label='Tank Y边界')
# ax4.axhline(y=tank_y_max, color='gray', linestyle='--')
# ax4.set_xlabel('时间 (s)')
# ax4.set_ylabel('Y 位置 (m)')
# ax4.set_title('Y位置随时间变化')
# ax4.legend()
# ax4.grid(True)

# plt.tight_layout()
# plt.savefig('tank_dob.png', dpi=300, bbox_inches='tight')
# plt.show()
# print(f"轨迹可视化图已保存: tank_dob.png")
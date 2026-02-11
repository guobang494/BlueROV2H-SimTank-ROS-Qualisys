#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trajectory Tracking Performance Analysis Script
Analyze BlueROV2 DOB-MPC controller trajectory tracking performance
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy import signal
import os
import re
from datetime import datetime

class TrajectoryTrackingAnalyzer:
    def __init__(self, bag_file_path):
        self.bag_file_path = bag_file_path
        self.data = {}
        self.bag_timestamp = self._extract_bag_timestamp()
        
    def _extract_bag_timestamp(self):
        """Extract timestamp from bag filename"""
        # Extract filename from path
        bag_filename = os.path.basename(self.bag_file_path)
        
        # Try to extract timestamp from filename (format: bluerov2_dobmpc_YYYYMMDD_HHMMSS.bag)
        timestamp_pattern = r'(\d{8}_\d{6})'
        match = re.search(timestamp_pattern, bag_filename)
        
        if match:
            return match.group(1)
        else:
            # Fallback to current time if no timestamp found in filename
            return datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def _get_unique_filename(self, base_name, extension):
        """Generate unique filename with sequence number if file exists"""
        # First try without sequence number
        filename = f"{base_name}_{self.bag_timestamp}.{extension}"
        
        if not os.path.exists(filename):
            return filename
        
        # If file exists, add sequence number
        counter = 1
        while True:
            filename = f"{base_name}_{self.bag_timestamp}_{counter:02d}.{extension}"
            if not os.path.exists(filename):
                return filename
            counter += 1
        
    def extract_data(self):
        """Extract data from bag file"""
        print(f"Analyzing bag file: {self.bag_file_path}")
        print(f"Extracted timestamp from bag: {self.bag_timestamp}")
        
        bag = rosbag.Bag(self.bag_file_path)
        
        # Extract actual position data
        actual_positions = []
        actual_times = []
        
        # Extract reference position data  
        reference_positions = []
        reference_times = []
        
        # Extract thrust data
        thrust_data = {i: [] for i in range(8)}
        thrust_times = {i: [] for i in range(8)}
        
        # Extract control input data
        control_inputs = {i: [] for i in range(4)}
        control_times = {i: [] for i in range(4)}
        
        print("Extracting data...")
        
        for topic, msg, t in bag.read_messages():
            # Actual position
            if topic == '/bluerov2/pose_gt':
                actual_positions.append([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y, 
                    msg.pose.pose.position.z,
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ])
                actual_times.append(t.to_sec())
                
            # Reference position
            elif topic == '/bluerov2/mpc/reference':
                reference_positions.append([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ])
                reference_times.append(t.to_sec())
                
            # Thrust data
            elif topic.startswith('/bluerov2/thrusters/') and topic.endswith('/thrust'):
                thruster_id = int(topic.split('/')[-2])
                thrust_data[thruster_id].append(msg.data)
                thrust_times[thruster_id].append(t.to_sec())
                
            # Control input
            elif topic.startswith('/bluerov2/control_input/'):
                control_id = int(topic.split('/')[-1])
                control_inputs[control_id].append(msg.data)
                control_times[control_id].append(t.to_sec())
        
        bag.close()
        
        # Convert to numpy arrays
        self.data['actual'] = np.array(actual_positions)
        self.data['actual_times'] = np.array(actual_times)
        self.data['reference'] = np.array(reference_positions)
        self.data['reference_times'] = np.array(reference_times)
        self.data['thrust'] = {i: np.array(thrust_data[i]) for i in range(8)}
        self.data['thrust_times'] = {i: np.array(thrust_times[i]) for i in range(8)}
        self.data['control'] = {i: np.array(control_inputs[i]) for i in range(4)}
        self.data['control_times'] = {i: np.array(control_times[i]) for i in range(4)}
        
        print(f"Data extraction completed:")
        print(f"  Actual position data points: {len(self.data['actual'])}")
        print(f"  Reference position data points: {len(self.data['reference'])}")
        print(f"  Thrust data points: {[len(self.data['thrust'][i]) for i in range(8)]}")
        print(f"  Control input data points: {[len(self.data['control'][i]) for i in range(4)]}")
        
        # Validate data
        if len(self.data['actual']) == 0:
            print("WARNING: No actual position data found!")
        if len(self.data['reference']) == 0:
            print("WARNING: No reference position data found!")
        
    def calculate_tracking_errors(self):
        """Calculate trajectory tracking errors"""
        print("Calculating tracking errors...")
        
        if len(self.data['actual']) == 0 or len(self.data['reference']) == 0:
            print("ERROR: Cannot calculate errors - missing position data!")
            return
        
        # Time alignment
        min_time = max(self.data['actual_times'][0], self.data['reference_times'][0])
        max_time = min(self.data['actual_times'][-1], self.data['reference_times'][-1])
        
        # Interpolate to common time points
        common_times = np.linspace(min_time, max_time, 1000)
        
        actual_interp = np.zeros((len(common_times), 6))
        reference_interp = np.zeros((len(common_times), 6))
        
        for i in range(6):
            actual_interp[:, i] = np.interp(common_times, self.data['actual_times'], self.data['actual'][:, i])
            reference_interp[:, i] = np.interp(common_times, self.data['reference_times'], self.data['reference'][:, i])
        
        # Calculate errors
        position_errors = actual_interp[:, :3] - reference_interp[:, :3]
        velocity_errors = actual_interp[:, 3:] - reference_interp[:, 3:]
        
        # Calculate error statistics
        self.error_stats = {
            'position_rmse': np.sqrt(np.mean(position_errors**2, axis=0)),
            'position_mae': np.mean(np.abs(position_errors), axis=0),
            'position_max': np.max(np.abs(position_errors), axis=0),
            'velocity_rmse': np.sqrt(np.mean(velocity_errors**2, axis=0)),
            'velocity_mae': np.mean(np.abs(velocity_errors), axis=0),
            'velocity_max': np.max(np.abs(velocity_errors), axis=0),
            'total_position_error': np.sqrt(np.sum(position_errors**2, axis=1)),
            'times': common_times
        }
        
        print("Error statistics:")
        print(f"  Position RMSE (x,y,z): {self.error_stats['position_rmse']}")
        print(f"  Position MAE (x,y,z): {self.error_stats['position_mae']}")
        print(f"  Position Max Error (x,y,z): {self.error_stats['position_max']}")
        print(f"  Velocity RMSE (x,y,z): {self.error_stats['velocity_rmse']}")
        
    def analyze_thrust_performance(self):
        """Analyze thruster performance"""
        print("Analyzing thruster performance...")
        
        self.thrust_stats = {}
        for i in range(8):
            if len(self.data['thrust'][i]) > 0:
                thrust_values = self.data['thrust'][i]
                self.thrust_stats[i] = {
                    'mean': np.mean(thrust_values),
                    'std': np.std(thrust_values),
                    'max': np.max(thrust_values),
                    'min': np.min(thrust_values),
                    'rms': np.sqrt(np.mean(thrust_values**2))
                }
        
        print("Thruster performance statistics:")
        for i in range(8):
            if i in self.thrust_stats:
                print(f"  Thruster{i}: Mean={self.thrust_stats[i]['mean']:.3f}, "
                      f"RMS={self.thrust_stats[i]['rms']:.3f}, "
                      f"Max={self.thrust_stats[i]['max']:.3f}")
    
    def plot_performance(self):
        """Plot performance analysis graphs"""
        print("Generating performance analysis plots...")
        
        # Check if we have data to plot
        if len(self.data['actual']) == 0 or len(self.data['reference']) == 0:
            print("ERROR: Cannot generate plots - no data available!")
            return
        
        # Create main figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('BlueROV2 DOB-MPC Trajectory Tracking Performance Analysis', fontsize=16)
        
        # Create separate 3D trajectory figure
        fig_3d = plt.figure(figsize=(12, 8))
        ax_3d = fig_3d.add_subplot(111, projection='3d')
        
        # 1. Trajectory comparison plot
        ax1 = axes[0, 0]
        ax1.plot(self.data['reference'][:, 0], self.data['reference'][:, 1], 'b--', 
                linewidth=2, label='Reference Trajectory')
        ax1.plot(self.data['actual'][:, 0], self.data['actual'][:, 1], 'r-', 
                linewidth=1, label='Actual Trajectory')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('XY Plane Trajectory Comparison')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # 2. Position error time series
        ax2 = axes[0, 1]
        if hasattr(self, 'error_stats'):
            ax2.plot(self.error_stats['times'], self.error_stats['total_position_error'], 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Total Position Error (m)')
        ax2.set_title('Position Error Time Series')
        ax2.grid(True)
        
        # 3. Position errors for each axis
        ax3 = axes[1, 0]
        min_len = min(len(self.data['actual']), len(self.data['reference']))
        if min_len > 0:
            position_errors = self.data['actual'][:min_len, :3] - self.data['reference'][:min_len, :3]
            time_subset = self.data['actual_times'][:min_len]
            ax3.plot(time_subset, position_errors[:, 0], 'r-', label='X-axis Error')
            ax3.plot(time_subset, position_errors[:, 1], 'g-', label='Y-axis Error')
            ax3.plot(time_subset, position_errors[:, 2], 'b-', label='Z-axis Error')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Position Error (m)')
        ax3.set_title('Position Errors for Each Axis')
        ax3.legend()
        ax3.grid(True)
        
        # 4. Thruster forces
        ax4 = axes[1, 1]
        for i in range(8):
            if len(self.data['thrust'][i]) > 0:
                ax4.plot(self.data['thrust_times'][i], self.data['thrust'][i], 
                        label=f'Thruster{i}', linewidth=1)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Thrust (N)')
        ax4.set_title('Thruster Forces')
        ax4.legend()
        ax4.grid(True)
        
        # 5. Control inputs
        ax5 = axes[2, 0]
        for i in range(4):
            if len(self.data['control'][i]) > 0:
                ax5.plot(self.data['control_times'][i], self.data['control'][i], 
                        label=f'Control Input{i}', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Control Input')
        ax5.set_title('Control Input Signals')
        ax5.legend()
        ax5.grid(True)
        
        # 6. Error statistics bar chart
        ax6 = axes[2, 1]
        if hasattr(self, 'error_stats'):
            axes_labels = ['X-axis', 'Y-axis', 'Z-axis']
            x_pos = np.arange(3)
            ax6.bar(x_pos - 0.2, self.error_stats['position_rmse'], 0.4, 
                   label='RMSE', alpha=0.8)
            ax6.bar(x_pos + 0.2, self.error_stats['position_mae'], 0.4, 
                   label='MAE', alpha=0.8)
            ax6.set_xticks(x_pos)
            ax6.set_xticklabels(axes_labels)
        ax6.set_xlabel('Coordinate Axis')
        ax6.set_ylabel('Error (m)')
        ax6.set_title('Position Error Statistics')
        ax6.legend()
        ax6.grid(True)
        
        plt.tight_layout()
        
        # Save main plot with bag timestamp and sequence number
        plot_filename = self._get_unique_filename('tracking_performance_analysis', 'png')
        fig.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Performance analysis plot saved: {plot_filename}")
        
        # Plot 3D trajectory
        self._plot_3d_trajectory(ax_3d)
        
        # Save 3D plot with bag timestamp and sequence number
        plot_3d_filename = self._get_unique_filename('3d_trajectory_analysis', 'png')
        fig_3d.savefig(plot_3d_filename, dpi=300, bbox_inches='tight')
        print(f"3D trajectory plot saved: {plot_3d_filename}")
        
        plt.show()
        # fig_3d.show() # Commented out to prevent errors in headless environments
        
    def _plot_3d_trajectory(self, ax):
        """Plot 3D trajectory comparison"""
        # Get trajectory data
        ref_traj = self.data['reference']
        actual_traj = self.data['actual']
        
        if len(ref_traj) == 0 or len(actual_traj) == 0:
            print("WARNING: No trajectory data to plot in 3D")
            return
        
        # Plot reference trajectory
        ax.plot(ref_traj[:, 0], ref_traj[:, 1], ref_traj[:, 2], 
               'b--', linewidth=2, label='Reference Trajectory', alpha=0.8)
        
        # Plot actual trajectory
        ax.plot(actual_traj[:, 0], actual_traj[:, 1], actual_traj[:, 2], 
               'r-', linewidth=1.5, label='Actual Trajectory', alpha=0.8)
        
        # Mark start and end points
        ax.scatter(ref_traj[0, 0], ref_traj[0, 1], ref_traj[0, 2], 
                  c='green', s=100, marker='o', label='Start Point', zorder=5)
        ax.scatter(ref_traj[-1, 0], ref_traj[-1, 1], ref_traj[-1, 2], 
                  c='red', s=100, marker='s', label='End Point', zorder=5)
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Trajectory Tracking Performance')
        
        # Add legend
        ax.legend()
        
        # Compress Z axis for better visualization
        ax.set_zlim(-1, 0)
        
        # Add grid
        ax.grid(True, alpha=0.3)
        
    def generate_report(self):
        """Generate performance analysis report"""
        print("\n" + "="*60)
        print("Trajectory Tracking Performance Analysis Report")
        print("="*60)
        
        if len(self.data['actual']) == 0:
            print("ERROR: No data available for report generation!")
            return
        
        print(f"\n📊 Data Overview:")
        print(f"  Bag file timestamp: {self.bag_timestamp}")
        print(f"  Recording duration: {self.data['actual_times'][-1] - self.data['actual_times'][0]:.1f} seconds")
        print(f"  Actual position data points: {len(self.data['actual'])}")
        print(f"  Reference position data points: {len(self.data['reference'])}")
        
        if hasattr(self, 'error_stats'):
            print(f"\n🎯 Tracking Accuracy:")
            print(f"  Position RMSE: X={self.error_stats['position_rmse'][0]:.4f}m, "
                  f"Y={self.error_stats['position_rmse'][1]:.4f}m, "
                  f"Z={self.error_stats['position_rmse'][2]:.4f}m")
            print(f"  Position MAE: X={self.error_stats['position_mae'][0]:.4f}m, "
                  f"Y={self.error_stats['position_mae'][1]:.4f}m, "
                  f"Z={self.error_stats['position_mae'][2]:.4f}m")
            print(f"  Max Position Error: X={self.error_stats['position_max'][0]:.4f}m, "
                  f"Y={self.error_stats['position_max'][1]:.4f}m, "
                  f"Z={self.error_stats['position_max'][2]:.4f}m")
            
            print(f"\n⚡ Velocity Tracking:")
            print(f"  Velocity RMSE: X={self.error_stats['velocity_rmse'][0]:.4f}m/s, "
                  f"Y={self.error_stats['velocity_rmse'][1]:.4f}m/s, "
                  f"Z={self.error_stats['velocity_rmse'][2]:.4f}m/s")
            
            print(f"\n📈 Performance Assessment:")
            avg_position_error = np.mean(self.error_stats['position_rmse'])
            if avg_position_error < 0.1:
                performance_level = "Excellent"
            elif avg_position_error < 0.2:
                performance_level = "Good"
            elif avg_position_error < 0.5:
                performance_level = "Fair"
            else:
                performance_level = "Needs Improvement"
            
            print(f"  Average Position Error: {avg_position_error:.4f} m")
            print(f"  Performance Level: {performance_level}")
        
        print(f"\n🔧 Thruster Performance:")
        total_thrust_rms = 0
        for i in range(8):
            if i in self.thrust_stats:
                total_thrust_rms += self.thrust_stats[i]['rms']**2
        if total_thrust_rms > 0:
            total_thrust_rms = np.sqrt(total_thrust_rms)
            print(f"  Total Thrust RMS: {total_thrust_rms:.3f} N")
        else:
            print(f"  No thrust data available")
        
        print("="*60)

def main():
    import sys
    
    # Set font for plots
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # Check command-line arguments
    if len(sys.argv) > 1:
        bag_file = sys.argv[1]
    else:
        # Default bag file path
        bag_file = "/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag"
        print(f"Using default bag file: {bag_file}")
        print("Usage: python3 analyze_tracking_performance.py <bag_file_path>")
    
    # Check if file exists
    if not os.path.exists(bag_file):
        print(f"Error: bag file does not exist: {bag_file}")
        sys.exit(1)
    
    print(f"Analyzing bag file: {bag_file}")
    
    analyzer = TrajectoryTrackingAnalyzer(bag_file)
    analyzer.extract_data()
    analyzer.calculate_tracking_errors()
    analyzer.analyze_thrust_performance()
    analyzer.plot_performance()
    analyzer.generate_report()

if __name__ == "__main__":
    main() 
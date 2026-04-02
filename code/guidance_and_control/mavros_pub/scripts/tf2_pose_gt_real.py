#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import math
from tf.transformations import euler_from_quaternion

class TF2PoseGroundTruth:
    def __init__(self):
        rospy.init_node('tf2_pose_gt_real', anonymous=True)
        
        # Parameters
        self.target_frame = rospy.get_param('~child_frame_id', 'BlueROV2H')  # Rigid body name to track
        self.reference_frame = rospy.get_param('~frame_id', 'mocap')     # Reference frame
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)       # Publish rate in Hz
        self.pose_topic = rospy.get_param('~pose_topic', '/bluerov2_heavy')  # Topic prefix
        
        # Subscribe to /tf topic from ros_qualisys
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        # Publisher for ground truth pose (Odometry format, kept for compatibility)
        self.pose_pub = rospy.Publisher(self.pose_topic, Odometry, queue_size=10)
        
        # 8 Float64 publishers mapped to bluerov2_motion_control
        self.pub_pos_x   = rospy.Publisher(self.pose_topic + '/position/linear/x',  Float64, queue_size=10)
        self.pub_pos_y   = rospy.Publisher(self.pose_topic + '/position/linear/y',  Float64, queue_size=10)
        self.pub_pos_z   = rospy.Publisher(self.pose_topic + '/position/linear/z',  Float64, queue_size=10)
        self.pub_pos_roll = rospy.Publisher(self.pose_topic + '/position/angular/x', Float64, queue_size=10)
        self.pub_pos_pitch = rospy.Publisher(self.pose_topic + '/position/angular/y', Float64, queue_size=10)
        self.pub_pos_yaw = rospy.Publisher(self.pose_topic + '/position/angular/z', Float64, queue_size=10)


        self.pub_vel_x   = rospy.Publisher(self.pose_topic + '/velocity_ned/linear/x',  Float64, queue_size=10)
        self.pub_vel_y   = rospy.Publisher(self.pose_topic + '/velocity_ned/linear/y',  Float64, queue_size=10)
        self.pub_vel_z   = rospy.Publisher(self.pose_topic + '/velocity_ned/linear/z',  Float64, queue_size=10)
        self.pub_vel_roll = rospy.Publisher(self.pose_topic + '/velocity_ned/angular/x', Float64, queue_size=10)
        self.pub_vel_pitch = rospy.Publisher(self.pose_topic + '/velocity_ned/angular/y', Float64, queue_size=10)
        self.pub_vel_yaw = rospy.Publisher(self.pose_topic + '/velocity_ned/angular/z', Float64, queue_size=10)
        
        self.pub_vel_body_x   = rospy.Publisher(self.pose_topic + '/velocity_body_frame/linear/x',  Float64, queue_size=10)
        self.pub_vel_body_y   = rospy.Publisher(self.pose_topic + '/velocity_body_frame/linear/y',  Float64, queue_size=10)
        self.pub_vel_body_z   = rospy.Publisher(self.pose_topic + '/velocity_body_frame/linear/z',  Float64, queue_size=10)
        self.pub_vel_body_roll = rospy.Publisher(self.pose_topic + '/velocity_body_frame/angular/x', Float64, queue_size=10)
        self.pub_vel_body_pitch = rospy.Publisher(self.pose_topic + '/velocity_body_frame/angular/y', Float64, queue_size=10)
        self.pub_vel_body_yaw = rospy.Publisher(self.pose_topic + '/velocity_body_frame/angular/z', Float64, queue_size=10)


        # Variables for velocity calculation
        self.last_pose = None
        self.last_yaw = None
        self.last_time = None
        self.current_transform = None
        
        # Rate control
        self.rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"TF2 Pose Ground Truth Node started")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        rospy.loginfo(f"Subscribing to: /tf")
        rospy.loginfo(f"Publishing Odometry to: {self.pose_topic}")
        rospy.loginfo(f"Publishing Float64 to: {self.pose_topic}/position/... and {self.pose_topic}/velocity/...")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
    
    def tf_callback(self, tf_msg):
        """Callback for the /tf topic subscriber."""
        rospy.loginfo_throttle(5.0, f"Received tf message with {len(tf_msg.transforms)} transforms")
        
        # Find the transform from the reference frame to the target frame
        for transform in tf_msg.transforms:
            rospy.loginfo_throttle(5.0, f"Transform: frame_id={transform.header.frame_id}, child_frame_id={transform.child_frame_id}")
            
            if transform.header.frame_id == self.reference_frame and transform.child_frame_id == self.target_frame:
                self.current_transform = transform
                rospy.loginfo_throttle(5.0, f"Found matching transform for {self.target_frame}!")
                break
        else:
            rospy.logwarn_throttle(5.0, f"No matching transform found. Looking for: frame_id={self.reference_frame}, child_frame_id={self.target_frame}")
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Extract yaw angle (rad) from quaternion."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_multiply(self, q1, q2):
        """Quaternion multiplication in [x, y, z, w] format."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def calculate_velocity(self, current_pose, current_yaw, current_time):
        """Compute linear and angular velocity."""
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.0]
        
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                # Compute linear velocity
                linear_vel[0] = (current_pose.position.x - self.last_pose.position.x) / dt
                linear_vel[1] = (current_pose.position.y - self.last_pose.position.y) / dt
                linear_vel[2] = (current_pose.position.z - self.last_pose.position.z) / dt
                
                # Compute angular velocity using quaternion differencing
                q_curr = np.array([current_pose.orientation.x, 
                                   current_pose.orientation.y, 
                                   current_pose.orientation.z, 
                                   current_pose.orientation.w])
                q_last = np.array([self.last_pose.orientation.x, 
                                   self.last_pose.orientation.y, 
                                   self.last_pose.orientation.z, 
                                   self.last_pose.orientation.w])
                
                # Keep quaternion sign consistent (q and -q represent the same rotation)
                if np.dot(q_curr, q_last) < 0:
                    q_last = -q_last
                
                # Compute relative rotation quaternion: q_rel = q_curr * q_last^(-1)
                q_last_inv = np.array([-q_last[0], -q_last[1], -q_last[2], q_last[3]])
                q_rel = self.quaternion_multiply(q_curr, q_last_inv)
                
                # Extract angular velocity from relative quaternion (axis-angle method)
                angle = 2.0 * np.arccos(np.clip(q_rel[3], -1.0, 1.0))
                if angle > 1e-6:
                    sin_half_angle = np.sin(angle / 2.0)
                    axis = q_rel[:3] / sin_half_angle
                    angular_vel = (axis * angle / dt).tolist()
                else:
                    angular_vel = [0.0, 0.0, 0.0]
        
        # Compute yaw rate separately via finite difference (more stable for Float64 output)
        yaw_rate = 0.0
        if self.last_yaw is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                yaw_rate = self.normalize_angle(current_yaw - self.last_yaw) / dt
        
        return linear_vel, angular_vel, yaw_rate
    
    def run(self):
        """Main loop."""
        while not rospy.is_shutdown():
            try:
                # Check if we have received tf data
                if self.current_transform is None:
                    rospy.logwarn_throttle(1.0, f"Waiting for tf data for frame: {self.target_frame}")
                    self.rate.sleep()
                    continue
                
                # Create Odometry message
                odom_msg = Odometry()
                
                # Fill message header
                odom_msg.header.stamp = self.current_transform.header.stamp
                odom_msg.header.frame_id = self.reference_frame
                odom_msg.child_frame_id = self.target_frame
                
                # Fill position
                odom_msg.pose.pose.position.x = self.current_transform.transform.translation.x
                odom_msg.pose.pose.position.y = self.current_transform.transform.translation.y
                odom_msg.pose.pose.position.z = self.current_transform.transform.translation.z
                
                # Fill orientation
                odom_msg.pose.pose.orientation.x = self.current_transform.transform.rotation.x
                odom_msg.pose.pose.orientation.y = self.current_transform.transform.rotation.y
                odom_msg.pose.pose.orientation.z = self.current_transform.transform.rotation.z
                odom_msg.pose.pose.orientation.w = self.current_transform.transform.rotation.w
                
                # Extract yaw from quaternion
                yaw = self.quaternion_to_yaw(
                    odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w
                )
                
                # Compute velocities
                current_time = self.current_transform.header.stamp
                linear_vel, angular_vel, yaw_rate = self.calculate_velocity(
                    odom_msg.pose.pose, yaw, current_time
                )
                
                # Fill velocity fields
                odom_msg.twist.twist.linear.x = linear_vel[0]
                odom_msg.twist.twist.linear.y = linear_vel[1]
                odom_msg.twist.twist.linear.z = linear_vel[2]
                odom_msg.twist.twist.angular.x = angular_vel[0]
                odom_msg.twist.twist.angular.y = angular_vel[1]
                odom_msg.twist.twist.angular.z = angular_vel[2]
                
                print("Current x_dot = " + str(linear_vel[0]))

                # Set covariance matrices
                odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                          0, 0.01, 0, 0, 0, 0,
                                          0, 0, 0.01, 0, 0, 0,
                                          0, 0, 0, 0.01, 0, 0,
                                          0, 0, 0, 0, 0.01, 0,
                                          0, 0, 0, 0, 0, 0.01]
                
                odom_msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                           0, 0.1, 0, 0, 0, 0,
                                           0, 0, 0.1, 0, 0, 0,
                                           0, 0, 0, 0.1, 0, 0,
                                           0, 0, 0, 0, 0.1, 0,
                                           0, 0, 0, 0, 0, 0.1]
                
                # Velocities from odometry (in NED/inertial frame)
                tlin = odom_msg.twist.twist.linear
                tang = odom_msg.twist.twist.angular

                q = odom_msg.pose.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                # Rotate NED/inertial vectors to body frame with ZYX convention.
                cphi = math.cos(roll)
                sphi = math.sin(roll)
                cth = math.cos(pitch)
                sth = math.sin(pitch)
                cps = math.cos(yaw)
                sps = math.sin(yaw)

                r11 = cth * cps
                r12 = cth * sps
                r13 = -sth
                r21 = sphi * sth * cps - cphi * sps
                r22 = sphi * sth * sps + cphi * cps
                r23 = sphi * cth
                r31 = cphi * sth * cps + sphi * sps
                r32 = cphi * sth * sps - sphi * cps
                r33 = cphi * cth

                vbx = r11 * tlin.x + r12 * tlin.y + r13 * tlin.z
                vby = r21 * tlin.x + r22 * tlin.y + r23 * tlin.z
                vbz = r31 * tlin.x + r32 * tlin.y + r33 * tlin.z

                wbx = r11 * tang.x + r12 * tang.y + r13 * tang.z
                wby = r21 * tang.x + r22 * tang.y + r23 * tang.z
                wbz = r31 * tang.x + r32 * tang.y + r33 * tang.z

                self.pub_vel_body_x.publish(Float64(vbx))
                self.pub_vel_body_y.publish(Float64(vby))
                self.pub_vel_body_z.publish(Float64(vbz))

                self.pub_vel_body_roll.publish(Float64(wbx))
                self.pub_vel_body_pitch.publish(Float64(wby))
                self.pub_vel_body_yaw.publish(Float64(wbz))


                


                # Publish Odometry (kept for compatibility)
                self.pose_pub.publish(odom_msg)
                
                # Publish 8 Float64 topics mapped to bluerov2_motion_control
                self.pub_pos_x.publish(Float64(data=odom_msg.pose.pose.position.x))
                self.pub_pos_y.publish(Float64(data=odom_msg.pose.pose.position.y))
                self.pub_pos_z.publish(Float64(data=odom_msg.pose.pose.position.z))
                self.pub_pos_roll.publish(Float64(0.0))  # TODO implement
                self.pub_pos_pitch.publish(Float64(0.0))  # TODO implement
                self.pub_pos_yaw.publish(Float64(data=yaw))

                self.pub_vel_x.publish(Float64(data=linear_vel[0]))
                self.pub_vel_y.publish(Float64(data=linear_vel[1]))
                self.pub_vel_z.publish(Float64(data=linear_vel[2]))
                self.pub_vel_roll.publish(Float64(data=0.0))
                self.pub_vel_pitch.publish(Float64(data=0.0))
                self.pub_vel_yaw.publish(Float64(data=yaw_rate))
                
                # Update historical data for velocity calculation
                self.last_pose = odom_msg.pose.pose
                self.last_yaw = yaw
                self.last_time = current_time
                
                # Logging
                rospy.loginfo_throttle(2.0, 
                    f"Pos: x={odom_msg.pose.pose.position.x:.3f} y={odom_msg.pose.pose.position.y:.3f} "
                    f"z={odom_msg.pose.pose.position.z:.3f} yaw={math.degrees(yaw):.1f}deg")
                rospy.loginfo_throttle(2.0,
                    f"Vel: vx={linear_vel[0]:.3f} vy={linear_vel[1]:.3f} "
                    f"vz={linear_vel[2]:.3f} vyaw={yaw_rate:.3f}")
                
            except Exception as e:
                rospy.logerr(f"Unexpected error: {str(e)}")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TF2PoseGroundTruth()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF2 Pose Ground Truth node terminated.")
    except Exception as e:
        rospy.logerr(f"Failed to start TF2 Pose Ground Truth node: {str(e)}")

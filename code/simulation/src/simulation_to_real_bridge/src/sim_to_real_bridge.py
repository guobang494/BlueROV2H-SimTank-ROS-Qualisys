#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


class SimToRealBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/bluerov2/pose_gt_ned")

        # Position publishers
        self.pub_pos = {
            "x": rospy.Publisher("/bluerov2_heavy/position/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/position/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/position/linear/z", Float64, queue_size=10),
        }

        # Euler publishers
        self.pub_ang = {
            "x": rospy.Publisher("/bluerov2_heavy/position/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/position/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/position/angular/z", Float64, queue_size=10),
        }

        # Velocity publishers
        self.pub_v_lin = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity/linear/z", Float64, queue_size=10),
        }
        self.pub_v_ang = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity/angular/z", Float64, queue_size=10),
        }

        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.cb_odom, queue_size=10)

        rospy.loginfo("sim_to_real_bridge started")
        rospy.loginfo("  odom_topic=%s", self.odom_topic)

    def cb_odom(self, msg: Odometry):
        # position
        p = msg.pose.pose.position
        self.pub_pos["x"].publish(Float64(p.x))
        self.pub_pos["y"].publish(Float64(p.y))
        self.pub_pos["z"].publish(Float64(p.z))
simulation_to_real_bridge
        # orientation
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pub_ang["x"].publish(Float64(roll))
        self.pub_ang["y"].publish(Float64(pitch))
        self.pub_ang["z"].publish(Float64(yaw))

        # velocities
        tlin = msg.twist.twist.linear
        tang = msg.twist.twist.angular

        self.pub_v_lin["x"].publish(Float64(tlin.x))
        self.pub_v_lin["y"].publish(Float64(tlin.y))
        self.pub_v_lin["z"].publish(Float64(tlin.z))

        self.pub_v_ang["x"].publish(Float64(tang.x))
        self.pub_v_ang["y"].publish(Float64(tang.y))
        self.pub_v_ang["z"].publish(Float64(tang.z))

if __name__ == "__main__":
    rospy.init_node("sim_to_real_bridge")
    SimToRealBridge()
    rospy.spin()

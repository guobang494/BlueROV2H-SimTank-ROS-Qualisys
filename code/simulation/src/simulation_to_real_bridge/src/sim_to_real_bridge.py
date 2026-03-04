#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
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

        # NED/inertial velocity publishers (as provided by pose_gt_ned odometry)
        self.pub_v_ned_lin = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/linear/z", Float64, queue_size=10),
        }
        self.pub_v_ned_ang = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity_ned_frame/angular/z", Float64, queue_size=10),
        }

        # Body-fixed velocity publishers
        self.pub_v_body_lin = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/linear/z", Float64, queue_size=10),
        }
        self.pub_v_body_ang = {
            "x": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/bluerov2_heavy/velocity_body_frame/angular/z", Float64, queue_size=10),
        }

        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.cb_odom, queue_size=10)

        rospy.loginfo("[sim_to_real_bridge] node started")
        rospy.loginfo("  odom_topic=%s", self.odom_topic)

    def cb_odom(self, msg: Odometry):
        # position
        p = msg.pose.pose.position
        self.pub_pos["x"].publish(Float64(p.x))
        self.pub_pos["y"].publish(Float64(p.y))
        self.pub_pos["z"].publish(Float64(p.z))

        # orientation
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pub_ang["x"].publish(Float64(roll))
        self.pub_ang["y"].publish(Float64(pitch))
        self.pub_ang["z"].publish(Float64(yaw))

        # Velocities from odometry (in NED/inertial frame)
        tlin = msg.twist.twist.linear
        tang = msg.twist.twist.angular

        self.pub_v_ned_lin["x"].publish(Float64(tlin.x))
        self.pub_v_ned_lin["y"].publish(Float64(tlin.y))
        self.pub_v_ned_lin["z"].publish(Float64(tlin.z))

        self.pub_v_ned_ang["x"].publish(Float64(tang.x))
        self.pub_v_ned_ang["y"].publish(Float64(tang.y))
        self.pub_v_ned_ang["z"].publish(Float64(tang.z))

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

        self.pub_v_body_lin["x"].publish(Float64(vbx))
        self.pub_v_body_lin["y"].publish(Float64(vby))
        self.pub_v_body_lin["z"].publish(Float64(vbz))

        self.pub_v_body_ang["x"].publish(Float64(wbx))
        self.pub_v_body_ang["y"].publish(Float64(wby))
        self.pub_v_body_ang["z"].publish(Float64(wbz))

if __name__ == "__main__":
    rospy.init_node("sim_to_real_bridge")
    SimToRealBridge()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

class FlipYZOdom:
    def __init__(self):
        in_topic  = rospy.get_param("~in_topic",  "/bluerov2/pose_gt")
        out_topic = rospy.get_param("~out_topic", "/bluerov2_temp/pose_gt_ned")

        self.pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, Odometry, self.cb, queue_size=10)

    def cb(self, msg: Odometry):
        # Copy message (we’ll modify fields in-place on a new instance)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        out.pose = msg.pose
        out.twist = msg.twist

        # Flip signs Y,Z
        out.pose.pose.position.y *= -1.0
        out.pose.pose.position.z *= -1.0

        # Flip orientation quaternion components y,z (consistent with axis sign flip)
        out.pose.pose.orientation.y *= -1.0
        out.pose.pose.orientation.z *= -1.0

        # Flip linear velocity Y,Z
        out.twist.twist.linear.y *= -1.0
        out.twist.twist.linear.z *= -1.0

        # Flip angular velocity Y,Z (often desired for consistency; remove if you don't want it)
        out.twist.twist.angular.y *= -1.0
        out.twist.twist.angular.z *= -1.0

        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("align_ned_odom")
    FlipYZOdom()
    rospy.spin()

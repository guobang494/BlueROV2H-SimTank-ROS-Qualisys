#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header

SRC_FRAME = 'world'
DST_FRAME = 'world_ned'
IN_TOPIC  = '/bluerov2/pose_gt'
OUT_TOPIC = '/bluerov2/pose_gt_ned'

class OdomNEDRepublisher:
    def __init__(self):
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)
        self.pub   = rospy.Publisher(OUT_TOPIC, Odometry, queue_size=10)
        self.sub   = rospy.Subscriber(IN_TOPIC, Odometry, self.cb, queue_size=10)

    def cb(self, msg):
        # Build stamped pose & twist in SRC_FRAME
        hdr = Header(stamp=msg.header.stamp, frame_id=SRC_FRAME)

        ps = PoseStamped()
        ps.header = hdr
        ps.pose   = msg.pose.pose

        ts = TwistStamped()
        ts.header = hdr
        ts.twist  = msg.twist.twist

        try:
            # Transform pose to NED
            xform = self.tfbuf.lookup_transform(DST_FRAME, SRC_FRAME, msg.header.stamp, rospy.Duration(0.2))
            ps_ned = tf2_geometry_msgs.do_transform_pose(ps, xform)
            # Transform twist (rotation only; assumes both frames share origin)
            ts_ned = tf2_geometry_msgs.do_transform_twist(ts, xform)

            out = Odometry()
            out.header.stamp    = msg.header.stamp
            out.header.frame_id = DST_FRAME
            out.child_frame_id  = msg.child_frame_id.replace('base_link', 'base_link_ned') if 'base_link' in msg.child_frame_id else msg.child_frame_id

            out.pose.pose  = ps_ned.pose
            out.twist.twist = ts_ned.twist
            # Preserve covariances if present (no rotation of covariances here; add if needed)
            out.pose.covariance  = msg.pose.covariance
            out.twist.covariance = msg.twist.covariance

            self.pub.publish(out)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "Waiting TF %s->%s: %s", SRC_FRAME, DST_FRAME, str(e))

if __name__ == '__main__':
    rospy.init_node('pose_gt_to_ned')
    OdomNEDRepublisher()
    rospy.loginfo("Republishing %s@%s -> %s as %s", IN_TOPIC, SRC_FRAME, DST_FRAME, OUT_TOPIC)
    rospy.spin()

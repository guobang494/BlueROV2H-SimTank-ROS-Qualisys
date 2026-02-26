#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_multiply


SRC_FRAME = 'world'   # pose header frame of incoming odom (map/odom/etc.)
DST_FRAME = 'world_ned'   # desired outgoing frame
IN_TOPIC  = '/bluerov2/pose_gt'
OUT_TOPIC = '/bluerov2/pose_gt_ned'

# Attempted correction as the observed map was found to be: 
# Your example "ground truth" mapping for position was:
#   x_dst =  y_src
#   y_dst = -x_src
#   z_dst =  z_src
#
Q_CORR = (0.0, 0.0, -0.7071067811865476, 0.7071067811865476) # this represents a -90deg rotation around z-axis of the Gazebo world frame


def apply_corr_vec(v):
    """Apply the fixed axis mapping matching your ground truth: (x,y,z)->(y,-x,z)."""
    x, y, z = v
    return (y, -x, z)


def apply_corr_quat(q_xyzw):
    """
    Apply the fixed correction rotation to an orientation quaternion.
    For active rotations: q_out = q_corr ⊗ q_in
    """
    return quaternion_multiply(Q_CORR, q_xyzw)


def transform_twist(tfbuf, twist, stamp, src_frame, dst_frame):
    """
    Properly transform a twist with tf2.
    NOTE: Odometry.twist is expressed in msg.child_frame_id (usually base_link),
    not in msg.header.frame_id.
    """
    ts = TwistStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = src_frame
    ts.twist = twist
    xform = tfbuf.lookup_transform(dst_frame, src_frame, stamp, rospy.Duration(0.2))
    return tf2_geometry_msgs.do_transform_twist(ts, xform).twist


class OdomNEDRepublisher:
    def __init__(self):
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)
        self.pub   = rospy.Publisher(OUT_TOPIC, Odometry, queue_size=10)
        self.sub   = rospy.Subscriber(IN_TOPIC, Odometry, self.cb, queue_size=10)

    def cb(self, msg):
        hdr = Header(stamp=msg.header.stamp, frame_id=SRC_FRAME)
        ps = PoseStamped()
        ps.header = hdr
        ps.pose = msg.pose.pose

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = DST_FRAME
        out.child_frame_id = msg.child_frame_id  # keep same child frame id

        try:
            # 1) Pose: TF transform SRC -> DST, then apply fixed correction
            xform_pose = self.tfbuf.lookup_transform(
                DST_FRAME, SRC_FRAME, msg.header.stamp, rospy.Duration(0.2)
            )
            ps_dst = tf2_geometry_msgs.do_transform_pose(ps, xform_pose)

            # Apply fixed correction to pose (matches your ground-truth mapping)
            # Position
            p = ps_dst.pose.position
            px, py, pz = apply_corr_vec((p.x, p.y, p.z))
            ps_dst.pose.position.x = px
            ps_dst.pose.position.y = py
            ps_dst.pose.position.z = pz

            # Orientation
            o = ps_dst.pose.orientation
            q_in = (o.x, o.y, o.z, o.w)
            q_out = apply_corr_quat(q_in)
            ps_dst.pose.orientation.x = q_out[0]
            ps_dst.pose.orientation.y = q_out[1]
            ps_dst.pose.orientation.z = q_out[2]
            ps_dst.pose.orientation.w = q_out[3]

            out.pose.pose = ps_dst.pose

            src_twist_frame = msg.child_frame_id
            dst_twist_frame = msg.child_frame_id

            tw_dst = transform_twist(
                self.tfbuf, msg.twist.twist, msg.header.stamp, src_twist_frame, dst_twist_frame
            )

            # Apply the same axis mapping to linear & angular components
            lv = apply_corr_vec((tw_dst.linear.x, tw_dst.linear.y, tw_dst.linear.z))
            av = apply_corr_vec((tw_dst.angular.x, tw_dst.angular.y, tw_dst.angular.z))
            tw_dst.linear.x,  tw_dst.linear.y,  tw_dst.linear.z  = lv
            tw_dst.angular.x, tw_dst.angular.y, tw_dst.angular.z = av

            out.twist.twist = tw_dst

            # Covariances copied as-is, if used they should be rotated TODO
            out.pose.covariance = msg.pose.covariance
            out.twist.covariance = msg.twist.covariance

            self.pub.publish(out)

        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(
                2.0,
                "Waiting TF (pose %s->%s, twist %s->%s): %s",
                SRC_FRAME, DST_FRAME, msg.child_frame_id, msg.child_frame_id, str(e)
            )


if __name__ == '__main__':
    rospy.init_node('pose_gt_to_ned')
    OdomNEDRepublisher()
    rospy.loginfo("Republishing %s@%s -> %s as %s", IN_TOPIC, SRC_FRAME, DST_FRAME, OUT_TOPIC)
    rospy.spin()
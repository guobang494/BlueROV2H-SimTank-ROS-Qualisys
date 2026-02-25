
#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_multiply, quaternion_conjugate

SRC_FRAME = 'world'
DST_FRAME = 'world_ned'
IN_TOPIC  = '/bluerov2/pose_gt'
OUT_TOPIC = '/bluerov2/pose_gt_ned'

def rotate_vector_by_quat(v, q):
    """
    Rotate a 3D vector v by quaternion q (x,y,z,w).
    We compute: v' = q * (v, 0) * q_conj
    """
    vx, vy, vz = v
    q_vec = (vx, vy, vz, 0.0)
    q_conj = quaternion_conjugate(q)
    q_tmp = quaternion_multiply(q, q_vec)
    q_rot = quaternion_multiply(q_tmp, q_conj)
    return (q_rot[0], q_rot[1], q_rot[2])

class OdomNEDRepublisher:
    def __init__(self):
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)
        self.pub   = rospy.Publisher(OUT_TOPIC, Odometry, queue_size=10)
        self.sub   = rospy.Subscriber(IN_TOPIC, Odometry, self.cb, queue_size=10)

    def cb(self, msg):
        # Build stamped pose in SRC_FRAME
        hdr = Header(stamp=msg.header.stamp, frame_id=SRC_FRAME)
        ps = PoseStamped()
        ps.header = hdr
        ps.pose   = msg.pose.pose

        try:
            # Lookup transform SRC->DST at the message time
            xform = self.tfbuf.lookup_transform(DST_FRAME, SRC_FRAME,
                                                msg.header.stamp, rospy.Duration(0.2))

            # 1) Transform pose with tf2 helper
            ps_ned = tf2_geometry_msgs.do_transform_pose(ps, xform)

            # 2) Rotate twist manually (only orientation part of transform)
            q = xform.transform.rotation
            q_xyzw = (q.x, q.y, q.z, q.w)

            v_lin = (msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z)
            v_ang = (msg.twist.twist.angular.x,
                     msg.twist.twist.angular.y,
                     msg.twist.twist.angular.z)

            v_lin_ned = rotate_vector_by_quat(v_lin, q_xyzw)
            v_ang_ned = rotate_vector_by_quat(v_ang, q_xyzw)

            # Publish Odometry in NED
            out = Odometry()
            out.header.stamp    = msg.header.stamp
            out.header.frame_id = DST_FRAME
            out.child_frame_id = msg.child_frame_id

            out.pose.pose = ps_ned.pose

            out.twist.twist.linear.x,  out.twist.twist.linear.y,  out.twist.twist.linear.z  = v_lin_ned
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = v_ang_ned

            # Copy covariances as-is (strictly, you’d rotate them too if you rely on them)
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


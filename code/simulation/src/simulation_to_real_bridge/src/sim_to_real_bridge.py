#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


class SimToRealBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/bluerov2/pose_gt_ned")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")

        self.model_index = int(rospy.get_param("~model_index", 2))
        self.model_name = rospy.get_param("~model_name", "")

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
        self.sub_states = rospy.Subscriber(self.model_states_topic, ModelStates, self.cb_states, queue_size=10)

        rospy.loginfo("sim_to_real_bridge started")
        rospy.loginfo("  odom_topic=%s", self.odom_topic)
        rospy.loginfo("  model_states_topic=%s", self.model_states_topic)

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pub_pos["x"].publish(Float64(p.x))
        self.pub_pos["y"].publish(Float64(p.y))
        self.pub_pos["z"].publish(Float64(p.z))

        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pub_ang["x"].publish(Float64(roll))
        self.pub_ang["y"].publish(Float64(pitch))
        self.pub_ang["z"].publish(Float64(yaw))

    def _resolve_model_index(self, names):
        if self.model_name:
            try:
                return names.index(self.model_name)
            except ValueError:
                rospy.logwarn_throttle(2.0, "Model name '%s' not found", self.model_name)
                return None
        if self.model_index < 0 or self.model_index >= len(names):
            rospy.logwarn_throttle(2.0, "model_index=%d out of range (len=%d)", self.model_index, len(names))
            return None
        return self.model_index

    def cb_states(self, msg: ModelStates):
        i = self._resolve_model_index(msg.name)
        if i is None:
            return
        t = msg.twist[i]

        # IMPORTANT: your world_ned is currently R = diag(1, -1, -1)
        # So vectors transform: x' = x, y' = -y, z' = -z
        self.pub_v_lin["x"].publish(Float64(t.linear.x))
        self.pub_v_lin["y"].publish(Float64(t.linear.y))
        self.pub_v_lin["z"].publish(Float64(t.linear.z))

        self.pub_v_ang["x"].publish(Float64(t.angular.x))
        self.pub_v_ang["y"].publish(Float64(t.angular.y))
        self.pub_v_ang["z"].publish(Float64(t.angular.z))


if __name__ == "__main__":
    rospy.init_node("sim_to_real_bridge")
    SimToRealBridge()
    rospy.spin()

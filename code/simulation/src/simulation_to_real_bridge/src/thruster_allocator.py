#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrusterAllocator:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_velocity")

        # Output topic base
        self.out_prefix = rospy.get_param("~out_prefix", "/bbb/ttt")
        self.n_thrusters = int(rospy.get_param("~n_thrusters", 8))

        # Load K (6x8) from param server
        K_list = rospy.get_param("~K", None)
        if K_list is None:
            raise RuntimeError("Parameter '~K' not found. Did you rosparam load the YAML?")

        self.K = np.array(K_list, dtype=float)
        if self.K.shape != (6, self.n_thrusters):
            raise RuntimeError(f"K must be shape (6,{self.n_thrusters}), got {self.K.shape}")

        # Precompute pseudoinverse: pinv(K) is (8x6) if K is (6x8)
        self.K_pinv = np.linalg.pinv(self.K)

        # Publishers for thrusters
        self.pubs = []
        for i in range(self.n_thrusters):
            topic = f"{self.out_prefix}/{i}/input"
            self.pubs.append(rospy.Publisher(topic, FloatStamped, queue_size=10))

        # Subscriber
        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cb_cmd, queue_size=10)

        rospy.loginfo("thruster_allocator started")
        rospy.loginfo("  cmd_topic=%s", self.cmd_topic)
        rospy.loginfo("  out_prefix=%s", self.out_prefix)
        rospy.loginfo("  K shape=%s", self.K.shape)

    def cb_cmd(self, msg: Twist):
        # tau = [vx, vy, vz, wx, wy, wz]^T
        tau = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ], dtype=float).reshape((6, 1))

        # F = pinv(K) * tau  -> (8x6)*(6x1) = (8x1)
        F = (self.K_pinv @ tau).reshape((self.n_thrusters,))

        now = rospy.Time.now()
        for i in range(self.n_thrusters):
            m = FloatStamped()
            m.header.stamp = now
            m.header.frame_id = ""
            m.data = float(F[i])
            self.pubs[i].publish(m)

if __name__ == "__main__":
    rospy.init_node("thruster_allocator")
    ThrusterAllocator()
    rospy.spin()


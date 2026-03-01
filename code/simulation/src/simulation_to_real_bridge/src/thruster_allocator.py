#!/usr/bin/env python3
import rospy
import numpy as np

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Float64


class ThrusterAllocator:
    def __init__(self):
    
        # Getting all parameters, either from the config file or from the launch file
        # Input topics (6 independent scalars)
        self.cmd_lin_x_topic = rospy.get_param("~cmd_lin_x_topic", "/bluerov2_heavy/cmd_velocity/linear/x")
        self.cmd_lin_y_topic = rospy.get_param("~cmd_lin_y_topic", "/bluerov2_heavy/cmd_velocity/linear/y")
        self.cmd_lin_z_topic = rospy.get_param("~cmd_lin_z_topic", "/bluerov2_heavy/cmd_velocity/linear/z")
        self.cmd_ang_x_topic = rospy.get_param("~cmd_ang_x_topic", "/bluerov2_heavy/cmd_velocity/angular/x")
        self.cmd_ang_y_topic = rospy.get_param("~cmd_ang_y_topic", "/bluerov2_heavy/cmd_velocity/angular/y")
        self.cmd_ang_z_topic = rospy.get_param("~cmd_ang_z_topic", "/bluerov2_heavy/cmd_velocity/angular/z")
        self.axis_names = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"]
        self.axis_topics = [
            self.cmd_lin_x_topic,
            self.cmd_lin_y_topic,
            self.cmd_lin_z_topic,
            self.cmd_ang_x_topic,
            self.cmd_ang_y_topic,
            self.cmd_ang_z_topic,
        ]

        # Per-axis gains (use this for sign flips without touching other nodes)
        self.g_lin_x = float(rospy.get_param("~gain_lin_x", 1.0))
        self.g_lin_y = float(rospy.get_param("~gain_lin_y", 1.0))
        self.g_lin_z = float(rospy.get_param("~gain_lin_z", 1.0))
        self.g_ang_x = float(rospy.get_param("~gain_ang_x", 1.0))
        self.g_ang_y = float(rospy.get_param("~gain_ang_y", 1.0))
        self.g_ang_z = float(rospy.get_param("~gain_ang_z", 1.0))
        if ((self.g_lin_x is None) or (self.g_lin_y is None) or (self.g_lin_z is None) or (self.g_ang_x is None) or (self.g_ang_y is None) or (self.g_ang_z is None)):
            raise RuntimeError("On the the '~g_lin' or '~g_ang' (multipliers) parameter was not found. Did you rosparam load the YAML?")

        # Output topic base
        self.out_prefix = rospy.get_param("~out_prefix", "/bluerov2/thrusters")
        self.n_thrusters = int(rospy.get_param("~n_thrusters", 8))
        if ((self.out_prefix is None) or (self.n_thrusters is None)):
            raise RuntimeError("Parameter '~out_prefix' or '~n_thrusters' not found. Did you rosparam load the YAML?")

        # Load K (6x8) from param server
        K_list = rospy.get_param("~K", None)
        if K_list is None:
            raise RuntimeError("Parameter '~K' not found. Did you rosparam load the YAML?")

        self.K = np.array(K_list, dtype=float)
        if self.K.shape != (6, self.n_thrusters):
            raise RuntimeError(f"K must be shape (6,{self.n_thrusters}), got {self.K.shape}")

        # Load thruster coefficient
        self.thruster_constant = rospy.get_param("~thruster_constant", None)
        if self.thruster_constant is None:
            raise RuntimeError("Parameter '~thruster_constant' not found. Did you rosparam load the YAML?")
        self.thruster_constant = float(self.thruster_constant)

        


        # Precompute pseudoinverse: pinv(K) is (8x6) if K is (6x8)
        self.K_pinv = np.linalg.pinv(self.K)

        # Publishers for thrusters
        self.pubs = []
        for i in range(self.n_thrusters):
            topic = f"{self.out_prefix}/{i}/input"
            self.pubs.append(rospy.Publisher(topic, FloatStamped, queue_size=10))

        # Latest commanded wrench components [vx, vy, vz, wx, wy, wz]
        self.cmd = np.zeros(6, dtype=float)
        self.have = np.zeros(6, dtype=bool)

        # Subscribers (6 scalar channels)
        self.sub_lin_x = rospy.Subscriber(self.cmd_lin_x_topic, Float64, self.cb_lin_x, queue_size=10)
        self.sub_lin_y = rospy.Subscriber(self.cmd_lin_y_topic, Float64, self.cb_lin_y, queue_size=10)
        self.sub_lin_z = rospy.Subscriber(self.cmd_lin_z_topic, Float64, self.cb_lin_z, queue_size=10)
        self.sub_ang_x = rospy.Subscriber(self.cmd_ang_x_topic, Float64, self.cb_ang_x, queue_size=10)
        self.sub_ang_y = rospy.Subscriber(self.cmd_ang_y_topic, Float64, self.cb_ang_y, queue_size=10)
        self.sub_ang_z = rospy.Subscriber(self.cmd_ang_z_topic, Float64, self.cb_ang_z, queue_size=10)

        # Publish thrusters at fixed rate (prevents spikes from partial updates)
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self._on_timer)

        rospy.loginfo("[thruster_allocator] node started")
        rospy.loginfo("  cmd_lin_x_topic=%s", self.cmd_lin_x_topic)
        rospy.loginfo("  cmd_lin_y_topic=%s", self.cmd_lin_y_topic)
        rospy.loginfo("  cmd_lin_z_topic=%s", self.cmd_lin_z_topic)
        rospy.loginfo("  cmd_ang_x_topic=%s", self.cmd_ang_x_topic)
        rospy.loginfo("  cmd_ang_y_topic=%s", self.cmd_ang_y_topic)
        rospy.loginfo("  cmd_ang_z_topic=%s", self.cmd_ang_z_topic)
        rospy.loginfo("  gains lin=[%s,%s,%s] ang=[%s,%s,%s]",
                      self.g_lin_x, self.g_lin_y, self.g_lin_z,
                      self.g_ang_x, self.g_ang_y, self.g_ang_z)
        rospy.loginfo("  out_prefix=%s", self.out_prefix)
        rospy.loginfo("  K shape=%s", self.K.shape)
        rospy.loginfo("  publish_rate_hz=%.1f", self.publish_rate_hz)

    def _on_timer(self, _evt):
        # Only publish once we have received at least one message for every axis
        if not np.all(self.have):
            missing = [
                f"{name} ({topic})"
                for idx, (name, topic) in enumerate(zip(self.axis_names, self.axis_topics))
                if not self.have[idx]
            ]
            rospy.logwarn_throttle(
                2.0,
                "Not all axes of the BlueROV2 were commanded. Missing: %s",
                ", ".join(missing),
            )
        self.publish_thrusters()

    def publish_thrusters(self):
        # tau = [vx, vy, vz, wx, wy, wz]^T
        tau = self.cmd.reshape((6, 1))

        # F = pinv(K) * tau
        F = (self.K_pinv @ tau).reshape((self.n_thrusters,))

        now = rospy.Time.now()
        for i in range(self.n_thrusters):
            m = FloatStamped()
            m.header.stamp = now
            m.header.frame_id = ""
            m.data = float(F[i] / self.thruster_constant)
            self.pubs[i].publish(m)

    def cb_lin_x(self, msg: Float64):
        self.cmd[0] = self.g_lin_x * msg.data
        self.have[0] = True

    def cb_lin_y(self, msg: Float64):
        self.cmd[1] = self.g_lin_y * msg.data
        self.have[1] = True

    def cb_lin_z(self, msg: Float64):
        self.cmd[2] = self.g_lin_z * msg.data
        self.have[2] = True

    def cb_ang_x(self, msg: Float64):
        self.cmd[3] = self.g_ang_x * msg.data
        self.have[3] = True

    def cb_ang_y(self, msg: Float64):
        self.cmd[4] = self.g_ang_y * msg.data
        self.have[4] = True

    def cb_ang_z(self, msg: Float64):
        self.cmd[5] = self.g_ang_z * msg.data
        self.have[5] = True


if __name__ == "__main__":
    rospy.init_node("thruster_allocator")
    ThrusterAllocator()
    rospy.spin()

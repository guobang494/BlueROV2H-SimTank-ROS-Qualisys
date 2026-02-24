#!/usr/bin/env python3
import rospy
import numpy as np

#from geometry_msgs.msg import Twist
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Float64



class ThrusterAllocator:
    def __init__(self):
        #self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_velocity")

        # Input topics (6 independent scalars - possibly can be improved)
        self.cmd_lin_x_topic = rospy.get_param("~cmd_lin_x_topic", "/bluerov2_heavy/cmd_velocity/linear/x")
        self.cmd_lin_y_topic = rospy.get_param("~cmd_lin_y_topic", "/bluerov2_heavy/cmd_velocity/linear/y")
        self.cmd_lin_z_topic = rospy.get_param("~cmd_lin_z_topic", "/bluerov2_heavy/cmd_velocity/linear/z")
        self.cmd_ang_x_topic = rospy.get_param("~cmd_ang_x_topic", "/bluerov2_heavy/cmd_velocity/angular/x")
        self.cmd_ang_y_topic = rospy.get_param("~cmd_ang_y_topic", "/bluerov2_heavy/cmd_velocity/angular/y")
        self.cmd_ang_z_topic = rospy.get_param("~cmd_ang_z_topic", "/bluerov2_heavy/cmd_velocity/angular/z")

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

        # Load thruster coefficient
        thruster_constant = rospy.get_param("~thruster_constant", None)
        if thruster_constant is None:
            raise RuntimeError("Parameter '~thruster_constant' not found. Did you rosparam load the YAML?")


        # Precompute pseudoinverse: pinv(K) is (8x6) if K is (6x8)
        self.K_pinv = np.linalg.pinv(self.K)

        # Publishers for thrusters
        self.pubs = []
        for i in range(self.n_thrusters):
            topic = f"{self.out_prefix}/{i}/input"
            self.pubs.append(rospy.Publisher(topic, FloatStamped, queue_size=10))

        # Latest commanded wrench components [vx, vy, vz, wx, wy, wz]
        self.cmd = np.zeros(6, dtype=float) # this is crucial as if one element of the wrench in not published, the 
        # pseudoinverse calculation can be executed

        # Subscribers (6 scalar channels)
        self.sub_lin_x = rospy.Subscriber(self.cmd_lin_x_topic, Float64, self.cb_lin_x, queue_size=10)
        self.sub_lin_y = rospy.Subscriber(self.cmd_lin_y_topic, Float64, self.cb_lin_y, queue_size=10)
        self.sub_lin_z = rospy.Subscriber(self.cmd_lin_z_topic, Float64, self.cb_lin_z, queue_size=10)
        self.sub_ang_x = rospy.Subscriber(self.cmd_ang_x_topic, Float64, self.cb_ang_x, queue_size=10)
        self.sub_ang_y = rospy.Subscriber(self.cmd_ang_y_topic, Float64, self.cb_ang_y, queue_size=10)
        self.sub_ang_z = rospy.Subscriber(self.cmd_ang_z_topic, Float64, self.cb_ang_z, queue_size=10)


        rospy.loginfo("thruster_allocator started")
        rospy.loginfo("  cmd_lin_x_topic=%s", self.cmd_lin_x_topic)
        rospy.loginfo("  cmd_lin_y_topic=%s", self.cmd_lin_y_topic)
        rospy.loginfo("  cmd_lin_z_topic=%s", self.cmd_lin_z_topic)
        rospy.loginfo("  cmd_ang_x_topic=%s", self.cmd_ang_x_topic)
        rospy.loginfo("  cmd_ang_y_topic=%s", self.cmd_ang_y_topic)
        rospy.loginfo("  cmd_ang_z_topic=%s", self.cmd_ang_z_topic)
        rospy.loginfo("  out_prefix=%s", self.out_prefix)
        rospy.loginfo("  K shape=%s", self.K.shape)

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
            m.data = float(F[i]/thruster_constant)
            self.pubs[i].publish(m)

    def cb_lin_x(self, msg: Float64):
        self.cmd[0] = msg.data
        self.publish_thrusters()

    def cb_lin_y(self, msg: Float64):
        self.cmd[1] = msg.data
        self.publish_thrusters()

    def cb_lin_z(self, msg: Float64):
        self.cmd[2] = msg.data
        self.publish_thrusters()

    def cb_ang_x(self, msg: Float64):
        self.cmd[3] = msg.data
        self.publish_thrusters()

    def cb_ang_y(self, msg: Float64):
        self.cmd[4] = msg.data
        self.publish_thrusters()

    def cb_ang_z(self, msg: Float64):
        self.cmd[5] = msg.data
        self.publish_thrusters()

if __name__ == "__main__":
    rospy.init_node("thruster_allocator")
    ThrusterAllocator()
    rospy.spin()


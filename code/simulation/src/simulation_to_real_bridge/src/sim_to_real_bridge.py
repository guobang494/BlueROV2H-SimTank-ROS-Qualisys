#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from gazebo_msgs.msg import ModelStates

class SimToRealBridge:
    def __init__(self):
        # Inputs (remappable / configurable)
        self.odom_topic = rospy.get_param("~odom_topic", "/simulation/pose")
        self.euler_topic = rospy.get_param("~euler_topic", "/simulation/euler")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/simulation/model_states")

        # Model selection for ModelStates
        self.model_index = int(rospy.get_param("~model_index", 2))
        self.model_name = rospy.get_param("~model_name", "")  # if set, overrides index

        # Publishers: position
        self.pub_pos = {
            "x": rospy.Publisher("/real_position/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/real_position/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/real_position/linear/z", Float64, queue_size=10),
        }

        # Publishers: euler
        self.pub_ang = {
            "x": rospy.Publisher("/real_position/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/real_position/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/real_position/angular/z", Float64, queue_size=10),
        }

        # Publishers: velocities
        self.pub_v_lin = {
            "x": rospy.Publisher("/real_velocity/velocity/linear/x", Float64, queue_size=10),
            "y": rospy.Publisher("/real_velocity/velocity/linear/y", Float64, queue_size=10),
            "z": rospy.Publisher("/real_velocity/velocity/linear/z", Float64, queue_size=10),
        }
        self.pub_v_ang = {
            "x": rospy.Publisher("/real_velocity/velocity/angular/x", Float64, queue_size=10),
            "y": rospy.Publisher("/real_velocity/velocity/angular/y", Float64, queue_size=10),
            "z": rospy.Publisher("/real_velocity/velocity/angular/z", Float64, queue_size=10),
        }

        # Subscribers
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.cb_odom, queue_size=10)
        self.sub_euler = rospy.Subscriber(self.euler_topic, Vector3Stamped, self.cb_euler, queue_size=10)
        self.sub_states = rospy.Subscriber(self.model_states_topic, ModelStates, self.cb_states, queue_size=10)

        rospy.loginfo("sim_to_real_bridge started")
        rospy.loginfo("  odom_topic=%s", self.odom_topic)
        rospy.loginfo("  euler_topic=%s", self.euler_topic)
        rospy.loginfo("  model_states_topic=%s", self.model_states_topic)
        rospy.loginfo("  model_name=%s model_index=%d", self.model_name, self.model_index)

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pub_pos["x"].publish(Float64(p.x))
        self.pub_pos["y"].publish(Float64(p.y))
        self.pub_pos["z"].publish(Float64(p.z))

    def cb_euler(self, msg: Vector3Stamped):
        v = msg.vector
        self.pub_ang["x"].publish(Float64(v.x))
        self.pub_ang["y"].publish(Float64(v.y))
        self.pub_ang["z"].publish(Float64(v.z))

    def _resolve_model_index(self, names):
        if self.model_name:
            try:
                return names.index(self.model_name)
            except ValueError:
                rospy.logwarn_throttle(2.0, "Model name '%s' not found in ModelStates.name", self.model_name)
                return None
        # index mode
        if self.model_index < 0 or self.model_index >= len(names):
            rospy.logwarn_throttle(2.0, "model_index=%d out of range (len=%d)", self.model_index, len(names))
            return None
        return self.model_index

    def cb_states(self, msg: ModelStates):
        i = self._resolve_model_index(msg.name)
        if i is None:
            return
        t = msg.twist[i]

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


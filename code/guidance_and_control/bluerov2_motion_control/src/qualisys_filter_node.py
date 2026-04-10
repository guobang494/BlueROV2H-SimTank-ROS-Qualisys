#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TopicFilter:
    def __init__(self, alpha, max_step, is_angular):
        self.alpha = float(max(0.0, min(1.0, alpha)))
        self.max_step = float(max_step)
        self.is_angular = bool(is_angular)
        self.initialized = False
        self.filtered = 0.0

    def _delta(self, value):
        delta = value - self.filtered
        if self.is_angular:
            delta = wrap_to_pi(delta)
        return delta

    def update(self, value, use_filter):
        value = float(value)
        if not self.initialized:
            self.filtered = value
            self.initialized = True
            return value

        if not use_filter:
            self.filtered = value
            return value

        delta = self._delta(value)
        if self.max_step > 0.0 and abs(delta) > self.max_step:
            delta = math.copysign(self.max_step, delta)

        self.filtered = self.filtered + self.alpha * delta
        if self.is_angular:
            self.filtered = wrap_to_pi(self.filtered)
        return self.filtered


class QualisysFilterNode:
    def __init__(self):
        rospy.init_node("qualisys_filter_node")

        self.pose_topic = rospy.get_param("~pose_topic", "/bluerov2_heavy").rstrip("/")
        self.enable_filter = bool(
            rospy.get_param("~qualisys_filter/enable_filter_qualisys", False)
        )
        self.publish_legacy_topics = bool(
            rospy.get_param("~qualisys_filter/publish_legacy_topics", True)
        )

        self.alpha_pos_lin = float(
            rospy.get_param("~qualisys_filter/position_linear_alpha", 0.25)
        )
        self.alpha_pos_ang = float(
            rospy.get_param("~qualisys_filter/position_angular_alpha", 0.20)
        )
        self.alpha_vel_lin = float(
            rospy.get_param("~qualisys_filter/velocity_linear_alpha", 0.25)
        )
        self.alpha_vel_ang = float(
            rospy.get_param("~qualisys_filter/velocity_angular_alpha", 0.20)
        )

        self.max_step_pos_lin = float(
            rospy.get_param("~qualisys_filter/max_step_position_linear", 0.10)
        )
        self.max_step_pos_ang = float(
            rospy.get_param("~qualisys_filter/max_step_position_angular", 0.10)
        )
        self.max_step_vel_lin = float(
            rospy.get_param("~qualisys_filter/max_step_velocity_linear", 0.25)
        )
        self.max_step_vel_ang = float(
            rospy.get_param("~qualisys_filter/max_step_velocity_angular", 0.25)
        )

        self.filter_by_suffix = {}
        self.pub_filtered = {}
        self.pub_legacy = {}
        self.subscribers = []

        self._register("position/linear/x", self.alpha_pos_lin, self.max_step_pos_lin, False)
        self._register("position/linear/y", self.alpha_pos_lin, self.max_step_pos_lin, False)
        self._register("position/linear/z", self.alpha_pos_lin, self.max_step_pos_lin, False)
        self._register("position/angular/x", self.alpha_pos_ang, self.max_step_pos_ang, True)
        self._register("position/angular/y", self.alpha_pos_ang, self.max_step_pos_ang, True)
        self._register("position/angular/z", self.alpha_pos_ang, self.max_step_pos_ang, True)

        self._register("velocity_ned/linear/x", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned/linear/y", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned/linear/z", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned/angular/x", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_ned/angular/y", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_ned/angular/z", self.alpha_vel_ang, self.max_step_vel_ang, True)

        self._register("velocity_ned_frame/linear/x", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned_frame/linear/y", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned_frame/linear/z", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_ned_frame/angular/x", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_ned_frame/angular/y", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_ned_frame/angular/z", self.alpha_vel_ang, self.max_step_vel_ang, True)

        self._register("velocity_body_frame/linear/x", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_body_frame/linear/y", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_body_frame/linear/z", self.alpha_vel_lin, self.max_step_vel_lin, False)
        self._register("velocity_body_frame/angular/x", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_body_frame/angular/y", self.alpha_vel_ang, self.max_step_vel_ang, True)
        self._register("velocity_body_frame/angular/z", self.alpha_vel_ang, self.max_step_vel_ang, True)

        rospy.loginfo(
            "[qualisys_filter_node] started: enable_filter_qualisys=%s, publish_legacy_topics=%s",
            self.enable_filter,
            self.publish_legacy_topics,
        )
        rospy.loginfo(
            "[qualisys_filter_node] input suffix: _meas, output suffix: _filtered, base topic=%s",
            self.pose_topic,
        )

    def _register(self, suffix, alpha, max_step, is_angular):
        meas_topic = f"{self.pose_topic}/{suffix}_meas"
        filtered_topic = f"{self.pose_topic}/{suffix}_filtered"
        legacy_topic = f"{self.pose_topic}/{suffix}"

        self.filter_by_suffix[suffix] = TopicFilter(alpha, max_step, is_angular)
        self.pub_filtered[suffix] = rospy.Publisher(filtered_topic, Float64, queue_size=10)
        if self.publish_legacy_topics:
            self.pub_legacy[suffix] = rospy.Publisher(legacy_topic, Float64, queue_size=10)

        sub = rospy.Subscriber(
            meas_topic,
            Float64,
            self._make_callback(suffix),
            queue_size=10,
        )
        self.subscribers.append(sub)

    def _make_callback(self, suffix):
        def callback(msg):
            raw = float(msg.data)
            filt = self.filter_by_suffix[suffix].update(raw, self.enable_filter)

            self.pub_filtered[suffix].publish(Float64(filt))
            if self.publish_legacy_topics:
                legacy_value = filt if self.enable_filter else raw
                self.pub_legacy[suffix].publish(Float64(legacy_value))

        return callback


if __name__ == "__main__":
    QualisysFilterNode()
    rospy.spin()

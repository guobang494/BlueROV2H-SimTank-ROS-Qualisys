#!/usr/bin/env python
import rospy
import yaml
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class GuidanceLawNode:

    def __init__(self):
        rospy.init_node("guidance_law_node")

        # Load YAML file
        config_file = rospy.get_param("~config_file")
        with open(config_file, 'r') as f:
            params = yaml.safe_load(f)

        self.path_waypoint_following = params["path_waypoint_following"]
        self.gamma = params["gamma"]
        self.waypoints = params["waypoints"]
        self.waypoint_cycling_active = params["waypoint_cycling_active"]
        self.max_yaw_rate = float(params.get("max_yaw_rate", 0.4))  # [rad/s]
        self.yaw_hold_distance = float(params.get("yaw_hold_distance", self.gamma))  # this parameter bounds 
        # the attitude reference not to change from within a distance gamma from a waypoint
        self.interp_distance_threshold = float(params.get("interp_distance_threshold", 1.0))  # [m]
        self.interp_angle_threshold_deg = float(params.get("interp_angle_threshold_deg", 20.0))  # [deg]
        self.interp_angle_threshold = math.radians(self.interp_angle_threshold_deg)

        self.index = 0  # current waypoint index
        self.ref_yaw = 0.0
        self.fake_target = None
        self.pending_real_index = None

        # Current state
        self.pos_x_gazebo = 0.0
        self.pos_y_gazebo = 0.0
        self.pos_z_gazebo = 0.0
        self.roll_gazebo = 0.0
        self.pitch_gazebo = 0.0
        self.yaw_gazebo = 0.0        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Publishers: reference position & orientation
        self.pub_ref_x = rospy.Publisher("/bluerov2_heavy/reference_position/linear/x", Float64, queue_size=10)
        self.pub_ref_y = rospy.Publisher("/bluerov2_heavy/reference_position/linear/y", Float64, queue_size=10)
        self.pub_ref_z = rospy.Publisher("/bluerov2_heavy/reference_position/linear/z", Float64, queue_size=10)
        self.pub_ref_roll  = rospy.Publisher("/bluerov2_heavy/reference_position/angular/x", Float64, queue_size=10)
        self.pub_ref_pitch = rospy.Publisher("/bluerov2_heavy/reference_position/angular/y", Float64, queue_size=10)
        self.pub_ref_yaw   = rospy.Publisher("/bluerov2_heavy/reference_position/angular/z", Float64, queue_size=10)

        # Publishers: diagnostic topics
        self.pub_next_target = rospy.Publisher("/guidance_law_variables/next_target", Float64MultiArray, queue_size=10)
        self.pub_distance = rospy.Publisher("/guidance_law_variables/distance_to_target", Float64, queue_size=10)

        # Subscribers: state
        rospy.Subscriber("/bluerov2_heavy/position/linear/x", Float64, self.cb_pos_x)
        rospy.Subscriber("/bluerov2_heavy/position/linear/y", Float64, self.cb_pos_y)
        rospy.Subscriber("/bluerov2_heavy/position/linear/z", Float64, self.cb_pos_z)
        rospy.Subscriber("/bluerov2_heavy/position/angular/x", Float64, self.cb_roll)
        rospy.Subscriber("/bluerov2_heavy/position/angular/y", Float64, self.cb_pitch)
        rospy.Subscriber("/bluerov2_heavy/position/angular/z", Float64, self.cb_yaw)

        rospy.loginfo("[guidance_law] Node started.")
        self.loop()

    @staticmethod
    def wrap_to_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def distance_3d(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def compute_ref_attitude_to_target(self, tx, ty, tz):
        dx = tx - self.pos_x
        dy = ty - self.pos_y
        dz = tz - self.pos_z

        if abs(dx) + abs(dy) > 1e-9:
            ref_yaw = math.atan2(dy, dx)
        else:
            ref_yaw = self.ref_yaw

        ref_pitch = math.atan2(dz, math.sqrt(dx*dx + dy*dy))
        ref_roll = 0.0
        return ref_roll, ref_pitch, ref_yaw

    def should_insert_intermediate_target(self, current_wp, next_wp):
        leg_dist = self.distance_3d(current_wp, next_wp)
        if leg_dist <= self.interp_distance_threshold:
            return False

        _, ref_pitch_next, ref_yaw_next = self.compute_ref_attitude_to_target(
            next_wp[0], next_wp[1], next_wp[2]
        )

        d_roll = abs(self.wrap_to_pi(0.0 - self.roll))
        d_pitch = abs(self.wrap_to_pi(ref_pitch_next - self.pitch))
        d_yaw = abs(self.wrap_to_pi(ref_yaw_next - self.yaw))

        return (
            d_roll > self.interp_angle_threshold or
            d_pitch > self.interp_angle_threshold or
            d_yaw > self.interp_angle_threshold
        )

    def get_next_waypoint_index(self):
        if self.index < len(self.waypoints) - 1:
            return self.index + 1
        if self.waypoint_cycling_active and len(self.waypoints) > 0:
            return 0
        return None

    # --- Callbacks ---
    def cb_pos_x(self, msg): self.pos_x_gazebo = msg.data
    def cb_pos_y(self, msg): self.pos_y_gazebo = msg.data
    def cb_pos_z(self, msg): self.pos_z_gazebo = msg.data
    def cb_roll(self, msg):  self.roll = msg.data
    def cb_pitch(self, msg): self.pitch = msg.data
    def cb_yaw(self, msg):   self.yaw = msg.data

    # --- Main control loop ---
    def loop(self):
        rate = rospy.Rate(20)  # 20 Hz
        dt = 1.0 / 20.0
        while not rospy.is_shutdown():

            # TODO careful
            self.pos_x = self.pos_x_gazebo
            self.pos_y = self.pos_y_gazebo
            self.pos_z = self.pos_z_gazebo
            
            if self.fake_target is not None:
                target = self.fake_target
            else:
                target = self.waypoints[self.index]
            tx, ty, tz = target
            dx = tx - self.pos_x
            dy = ty - self.pos_y
            horizontal_dist = math.sqrt(dx*dx + dy*dy)

            # Compute distance to the next waypoint
            dist = math.sqrt((tx - self.pos_x)**2 +
                             (ty - self.pos_y)**2 +
                             (tz - self.pos_z)**2)

            # Publish diagnostic distance
            self.pub_distance.publish(dist)

            # Switch waypoint when the distance is < threshold
            if dist < self.gamma:
                if self.fake_target is not None:
                    # Intermediate target reached, now command the originally planned waypoint.
                    self.fake_target = None
                    if self.pending_real_index is not None:
                        self.index = self.pending_real_index
                        self.pending_real_index = None
                else:
                    next_index = self.get_next_waypoint_index()
                    if next_index is not None:
                        current_wp = self.waypoints[self.index]
                        next_wp = self.waypoints[next_index]
                        if self.should_insert_intermediate_target(current_wp, next_wp):
                            self.fake_target = [
                                0.5 * (current_wp[0] + next_wp[0]),
                                0.5 * (current_wp[1] + next_wp[1]),
                                0.5 * (current_wp[2] + next_wp[2]),
                            ]
                            self.pending_real_index = next_index
                            rospy.loginfo(
                                "[guidance_law] inserted intermediate target (%.3f, %.3f, %.3f) before waypoint %d",
                                self.fake_target[0], self.fake_target[1], self.fake_target[2], next_index
                            )
                        else:
                            self.index = next_index

                if self.fake_target is not None:
                    target = self.fake_target
                else:
                    target = self.waypoints[self.index]
                tx, ty, tz = target
                dx = tx - self.pos_x
                dy = ty - self.pos_y
                horizontal_dist = math.sqrt(dx*dx + dy*dy)

            # Compute reference orientation
            if horizontal_dist > 1e-6:
                raw_ref_yaw = math.atan2(dy, dx)
            else:
                raw_ref_yaw = self.ref_yaw

            # Keep yaw reference continuous and rate-limited across waypoint switches.
            if horizontal_dist < self.yaw_hold_distance:
                ref_yaw = self.ref_yaw
            else:
                yaw_err_ref = self.wrap_to_pi(raw_ref_yaw - self.ref_yaw)
                max_step = self.max_yaw_rate * dt
                yaw_step = max(-max_step, min(max_step, yaw_err_ref))
                ref_yaw = self.wrap_to_pi(self.ref_yaw + yaw_step)

            self.ref_yaw = ref_yaw
            # 
            # ref_pitch = math.atan2(tz - self.pos_z,
            #                        math.sqrt((tx - self.pos_x)**2 +
            #                                  (ty - self.pos_y)**2))
            

            ref_roll, ref_pitch, _ = self.compute_ref_attitude_to_target(tx, ty, tz)

            # Publish reference position
            self.pub_ref_x.publish(tx)
            self.pub_ref_y.publish(ty)
            self.pub_ref_z.publish(tz)

            # Publish reference orientation
            self.pub_ref_roll.publish(ref_roll)
            self.pub_ref_pitch.publish(ref_pitch)
            self.pub_ref_yaw.publish(ref_yaw)

            # Publish next target waypoint
            msg = Float64MultiArray()
            msg.data = [tx, ty, tz, ref_roll, ref_pitch, ref_yaw]
            self.pub_next_target.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        GuidanceLawNode()
    except rospy.ROSInterruptException:
        pass

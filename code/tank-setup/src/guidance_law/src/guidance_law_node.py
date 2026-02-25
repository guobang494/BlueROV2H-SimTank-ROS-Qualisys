#!/usr/bin/env python
import rospy
import yaml
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import csv
import os
from datetime import datetime



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

        self.index = 0  # current waypoint index

        # Current state
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

        # --- Logging setup ---
        log_dir = os.path.join(os.path.expanduser("~"), "guidance_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"guidance_log_{timestamp}.csv")

        self.log_file = open(self.log_path, "w")
        self.log_writer = csv.writer(self.log_file)

        # Write CSV header
        self.log_writer.writerow([
            "time",
            "pos_x", "pos_y", "pos_z",
            "roll", "pitch", "yaw",
            "target_x", "target_y", "target_z",
            "ref_roll", "ref_pitch", "ref_yaw",
            "distance_to_target",
            "waypoint_index",
            "waypoint_cycling_active"
        ])

        rospy.loginfo(f"[guidance_law] Logging to {self.log_path}")

        rospy.loginfo("[guidance_law] Node started.")
        self.loop()

    # --- Callbacks ---
    def cb_pos_x(self, msg): self.pos_x = msg.data
    def cb_pos_y(self, msg): self.pos_y = msg.data
    def cb_pos_z(self, msg): self.pos_z = msg.data
    def cb_roll(self, msg):  self.roll = msg.data
    def cb_pitch(self, msg): self.pitch = msg.data
    def cb_yaw(self, msg):   self.yaw = msg.data


    # Function to wrap the target reference angles to +/- pi to avoid commanding long rotations 
    def _wrap_pi(self, angle):
        # Wrap angle to (-pi, pi]
        return (angle + math.pi) % (2.0 * math.pi) - math.pi



    def log_data(self, target, ref_roll, ref_pitch, ref_yaw, distance):
        tx, ty, tz = target

        self.log_writer.writerow([
            rospy.get_time(),
            self.pos_x, self.pos_y, self.pos_z,
            self.roll, self.pitch, self.yaw,
            tx, ty, tz,
            ref_roll, ref_pitch, ref_yaw,
            distance,
            self.index,
            self.waypoint_cycling_active
        ])
        self.log_file.flush()




    # Main control loop
    def loop(self):
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():

            target = self.waypoints[self.index]
            tx, ty, tz = target

            # Compute distance to the next waypoint
            dist = math.sqrt((tx - self.pos_x)**2 +
                             (ty - self.pos_y)**2 +
                             (tz - self.pos_z)**2)

            # Publish diagnostic distance
            self.pub_distance.publish(dist)

            # Switch waypoint when the distance is < threshold
            if dist < self.gamma:
                if self.index < len(self.waypoints) - 1:
                    # Normal advance to next waypoint
                    self.index += 1
                elif self.waypoint_cycling_active:
                    # Restart from waypoint 0 if cycling enabled
                    self.index = 0

                target = self.waypoints[self.index]
                tx, ty, tz = target

            # Calculating difference (target-current position)
            dx = tx - self.pos_x
            dy = ty - self.pos_y
            dz = tz - self.pos_z

            # Compute reference orientation in the horizontal plane
            ref_yaw = - math.atan2(dy, dx)  # CAVEAT: the minus is because we have a NED convention

            # Wrap target yaw angle to avoid discontinuities and ensuring shortest-path relative to current yaw feedback
            ref_yaw = self._wrap_pi(self.yaw + self._wrap_pi(ref_yaw - self.yaw))

            # Compute reference orientation in the vertical plane
            ref_pitch = 0.0 # TODO set to zero temprarily math.atan2(dz, dx)

            ref_roll = 0.0  # optional: can be extended later

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

            # Log the data in a csv file
            self.log_data((tx, ty, tz), ref_roll, ref_pitch, ref_yaw, dist)

            rate.sleep()


if __name__ == "__main__":
    try:
        GuidanceLawNode()
    except rospy.ROSInterruptException:
        pass


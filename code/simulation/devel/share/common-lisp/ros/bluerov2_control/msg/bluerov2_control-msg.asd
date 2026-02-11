
(cl:in-package :asdf)

(defsystem "bluerov2_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
               :uuv_control_msgs-msg
)
  :components ((:file "_package")
    (:file "Autopilot" :depends-on ("_package_Autopilot"))
    (:file "_package_Autopilot" :depends-on ("_package"))
    (:file "ControlMode" :depends-on ("_package_ControlMode"))
    (:file "_package_ControlMode" :depends-on ("_package"))
    (:file "FollowWaypointsAction" :depends-on ("_package_FollowWaypointsAction"))
    (:file "_package_FollowWaypointsAction" :depends-on ("_package"))
    (:file "FollowWaypointsActionFeedback" :depends-on ("_package_FollowWaypointsActionFeedback"))
    (:file "_package_FollowWaypointsActionFeedback" :depends-on ("_package"))
    (:file "FollowWaypointsActionGoal" :depends-on ("_package_FollowWaypointsActionGoal"))
    (:file "_package_FollowWaypointsActionGoal" :depends-on ("_package"))
    (:file "FollowWaypointsActionResult" :depends-on ("_package_FollowWaypointsActionResult"))
    (:file "_package_FollowWaypointsActionResult" :depends-on ("_package"))
    (:file "FollowWaypointsFeedback" :depends-on ("_package_FollowWaypointsFeedback"))
    (:file "_package_FollowWaypointsFeedback" :depends-on ("_package"))
    (:file "FollowWaypointsGoal" :depends-on ("_package_FollowWaypointsGoal"))
    (:file "_package_FollowWaypointsGoal" :depends-on ("_package"))
    (:file "FollowWaypointsResult" :depends-on ("_package_FollowWaypointsResult"))
    (:file "_package_FollowWaypointsResult" :depends-on ("_package"))
  ))
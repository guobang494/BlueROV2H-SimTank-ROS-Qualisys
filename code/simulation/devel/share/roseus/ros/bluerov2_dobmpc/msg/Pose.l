;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_dobmpc::Pose)
  (if (not (find-package "BLUEROV2_DOBMPC"))
    (make-package "BLUEROV2_DOBMPC"))
  (shadow 'Pose (find-package "BLUEROV2_DOBMPC")))
(unless (find-package "BLUEROV2_DOBMPC::POSE")
  (make-package "BLUEROV2_DOBMPC::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bluerov2_dobmpc::Pose
  :super ros::object
  :slots (_header _current_pose _current_twist ))

(defmethod bluerov2_dobmpc::Pose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:current_pose __current_pose) ())
    ((:current_twist __current_twist) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _current_pose __current_pose)
   (setq _current_twist __current_twist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:current_pose
   (&rest __current_pose)
   (if (keywordp (car __current_pose))
       (send* _current_pose __current_pose)
     (progn
       (if __current_pose (setq _current_pose (car __current_pose)))
       _current_pose)))
  (:current_twist
   (&rest __current_twist)
   (if (keywordp (car __current_twist))
       (send* _current_twist __current_twist)
     (progn
       (if __current_twist (setq _current_twist (car __current_twist)))
       _current_twist)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose[] _current_pose
    (apply #'+ (send-all _current_pose :serialization-length)) 4
    ;; geometry_msgs/Twist[] _current_twist
    (apply #'+ (send-all _current_twist :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose[] _current_pose
     (write-long (length _current_pose) s)
     (dolist (elem _current_pose)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Twist[] _current_twist
     (write-long (length _current_twist) s)
     (dolist (elem _current_twist)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose[] _current_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current_pose (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _current_pose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Twist[] _current_twist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current_twist (let (r) (dotimes (i n) (push (instance geometry_msgs::Twist :init) r)) r))
     (dolist (elem- _current_twist)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get bluerov2_dobmpc::Pose :md5sum-) "a2658de99fec3ecc40cbc0db789c4bb3")
(setf (get bluerov2_dobmpc::Pose :datatype-) "bluerov2_dobmpc/Pose")
(setf (get bluerov2_dobmpc::Pose :definition-)
      "Header header
geometry_msgs/Pose[] current_pose
geometry_msgs/Twist[] current_twist
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :bluerov2_dobmpc/Pose "a2658de99fec3ecc40cbc0db789c4bb3")



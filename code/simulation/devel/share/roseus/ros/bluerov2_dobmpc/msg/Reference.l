;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_dobmpc::Reference)
  (if (not (find-package "BLUEROV2_DOBMPC"))
    (make-package "BLUEROV2_DOBMPC"))
  (shadow 'Reference (find-package "BLUEROV2_DOBMPC")))
(unless (find-package "BLUEROV2_DOBMPC::REFERENCE")
  (make-package "BLUEROV2_DOBMPC::REFERENCE"))

(in-package "ROS")
;;//! \htmlinclude Reference.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bluerov2_dobmpc::Reference
  :super ros::object
  :slots (_header _ref_pose _ref_twist ))

(defmethod bluerov2_dobmpc::Reference
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ref_pose __ref_pose) ())
    ((:ref_twist __ref_twist) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _ref_pose __ref_pose)
   (setq _ref_twist __ref_twist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ref_pose
   (&rest __ref_pose)
   (if (keywordp (car __ref_pose))
       (send* _ref_pose __ref_pose)
     (progn
       (if __ref_pose (setq _ref_pose (car __ref_pose)))
       _ref_pose)))
  (:ref_twist
   (&rest __ref_twist)
   (if (keywordp (car __ref_twist))
       (send* _ref_twist __ref_twist)
     (progn
       (if __ref_twist (setq _ref_twist (car __ref_twist)))
       _ref_twist)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose[] _ref_pose
    (apply #'+ (send-all _ref_pose :serialization-length)) 4
    ;; geometry_msgs/Twist[] _ref_twist
    (apply #'+ (send-all _ref_twist :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose[] _ref_pose
     (write-long (length _ref_pose) s)
     (dolist (elem _ref_pose)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Twist[] _ref_twist
     (write-long (length _ref_twist) s)
     (dolist (elem _ref_twist)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose[] _ref_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ref_pose (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _ref_pose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Twist[] _ref_twist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ref_twist (let (r) (dotimes (i n) (push (instance geometry_msgs::Twist :init) r)) r))
     (dolist (elem- _ref_twist)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get bluerov2_dobmpc::Reference :md5sum-) "a5a6b3393e50b440b27316a2d50af420")
(setf (get bluerov2_dobmpc::Reference :datatype-) "bluerov2_dobmpc/Reference")
(setf (get bluerov2_dobmpc::Reference :definition-)
      "Header header
geometry_msgs/Pose[] ref_pose
geometry_msgs/Twist[] ref_twist

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



(provide :bluerov2_dobmpc/Reference "a5a6b3393e50b440b27316a2d50af420")



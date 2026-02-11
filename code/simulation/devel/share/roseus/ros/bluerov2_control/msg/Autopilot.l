;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_control::Autopilot)
  (if (not (find-package "BLUEROV2_CONTROL"))
    (make-package "BLUEROV2_CONTROL"))
  (shadow 'Autopilot (find-package "BLUEROV2_CONTROL")))
(unless (find-package "BLUEROV2_CONTROL::AUTOPILOT")
  (make-package "BLUEROV2_CONTROL::AUTOPILOT"))

(in-package "ROS")
;;//! \htmlinclude Autopilot.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*DEPTH*" (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(shadow '*DEPTH* (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(defconstant bluerov2_control::Autopilot::*DEPTH* 0)
(intern "*ALT*" (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(shadow '*ALT* (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(defconstant bluerov2_control::Autopilot::*ALT* 1)
(intern "*NONE*" (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(shadow '*NONE* (find-package "BLUEROV2_CONTROL::AUTOPILOT"))
(defconstant bluerov2_control::Autopilot::*NONE* 2)

(defun bluerov2_control::Autopilot-to-symbol (const)
  (cond
        ((= const 0) 'bluerov2_control::Autopilot::*DEPTH*)
        ((= const 1) 'bluerov2_control::Autopilot::*ALT*)
        ((= const 2) 'bluerov2_control::Autopilot::*NONE*)
        (t nil)))

(defclass bluerov2_control::Autopilot
  :super ros::object
  :slots (_header _U _Z _psi _reference ))

(defmethod bluerov2_control::Autopilot
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:U __U) 0.0)
    ((:Z __Z) 0.0)
    ((:psi __psi) 0.0)
    ((:reference __reference) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _U (float __U))
   (setq _Z (float __Z))
   (setq _psi (float __psi))
   (setq _reference (round __reference))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:U
   (&optional __U)
   (if __U (setq _U __U)) _U)
  (:Z
   (&optional __Z)
   (if __Z (setq _Z __Z)) _Z)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _U
    8
    ;; float64 _Z
    8
    ;; float64 _psi
    8
    ;; uint8 _reference
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _U
       (sys::poke _U (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z
       (sys::poke _Z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _reference
       (write-byte _reference s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _U
     (setq _U (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z
     (setq _Z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _reference
     (setq _reference (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get bluerov2_control::Autopilot :md5sum-) "4b52700fd1487670315929e62e105ffb")
(setf (get bluerov2_control::Autopilot :datatype-) "bluerov2_control/Autopilot")
(setf (get bluerov2_control::Autopilot :definition-)
      "uint8 DEPTH=0
uint8 ALT=1
uint8 NONE=2
Header header
float64 U  # forward speed (m/s)
float64 Z  # vertical distance (m)
float64 psi  # Heading (ENU) (radians)
uint8 reference  # vertical distance reference type

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

")



(provide :bluerov2_control/Autopilot "4b52700fd1487670315929e62e105ffb")



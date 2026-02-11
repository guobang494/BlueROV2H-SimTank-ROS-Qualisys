; Auto-generated. Do not edit!


(cl:in-package bluerov2_control-msg)


;//! \htmlinclude Autopilot.msg.html

(cl:defclass <Autopilot> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (U
    :reader U
    :initarg :U
    :type cl:float
    :initform 0.0)
   (Z
    :reader Z
    :initarg :Z
    :type cl:float
    :initform 0.0)
   (psi
    :reader psi
    :initarg :psi
    :type cl:float
    :initform 0.0)
   (reference
    :reader reference
    :initarg :reference
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Autopilot (<Autopilot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Autopilot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Autopilot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-msg:<Autopilot> is deprecated: use bluerov2_control-msg:Autopilot instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Autopilot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:header-val is deprecated.  Use bluerov2_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'U-val :lambda-list '(m))
(cl:defmethod U-val ((m <Autopilot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:U-val is deprecated.  Use bluerov2_control-msg:U instead.")
  (U m))

(cl:ensure-generic-function 'Z-val :lambda-list '(m))
(cl:defmethod Z-val ((m <Autopilot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:Z-val is deprecated.  Use bluerov2_control-msg:Z instead.")
  (Z m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <Autopilot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:psi-val is deprecated.  Use bluerov2_control-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'reference-val :lambda-list '(m))
(cl:defmethod reference-val ((m <Autopilot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:reference-val is deprecated.  Use bluerov2_control-msg:reference instead.")
  (reference m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Autopilot>)))
    "Constants for message type '<Autopilot>"
  '((:DEPTH . 0)
    (:ALT . 1)
    (:NONE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Autopilot)))
    "Constants for message type 'Autopilot"
  '((:DEPTH . 0)
    (:ALT . 1)
    (:NONE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Autopilot>) ostream)
  "Serializes a message object of type '<Autopilot>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'U))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Autopilot>) istream)
  "Deserializes a message object of type '<Autopilot>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'U) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Autopilot>)))
  "Returns string type for a message object of type '<Autopilot>"
  "bluerov2_control/Autopilot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Autopilot)))
  "Returns string type for a message object of type 'Autopilot"
  "bluerov2_control/Autopilot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Autopilot>)))
  "Returns md5sum for a message object of type '<Autopilot>"
  "4b52700fd1487670315929e62e105ffb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Autopilot)))
  "Returns md5sum for a message object of type 'Autopilot"
  "4b52700fd1487670315929e62e105ffb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Autopilot>)))
  "Returns full string definition for message of type '<Autopilot>"
  (cl:format cl:nil "uint8 DEPTH=0~%uint8 ALT=1~%uint8 NONE=2~%Header header~%float64 U  # forward speed (m/s)~%float64 Z  # vertical distance (m)~%float64 psi  # Heading (ENU) (radians)~%uint8 reference  # vertical distance reference type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Autopilot)))
  "Returns full string definition for message of type 'Autopilot"
  (cl:format cl:nil "uint8 DEPTH=0~%uint8 ALT=1~%uint8 NONE=2~%Header header~%float64 U  # forward speed (m/s)~%float64 Z  # vertical distance (m)~%float64 psi  # Heading (ENU) (radians)~%uint8 reference  # vertical distance reference type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Autopilot>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Autopilot>))
  "Converts a ROS message object to a list"
  (cl:list 'Autopilot
    (cl:cons ':header (header msg))
    (cl:cons ':U (U msg))
    (cl:cons ':Z (Z msg))
    (cl:cons ':psi (psi msg))
    (cl:cons ':reference (reference msg))
))

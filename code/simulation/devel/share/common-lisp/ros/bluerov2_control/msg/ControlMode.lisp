; Auto-generated. Do not edit!


(cl:in-package bluerov2_control-msg)


;//! \htmlinclude ControlMode.msg.html

(cl:defclass <ControlMode> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControlMode (<ControlMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-msg:<ControlMode> is deprecated: use bluerov2_control-msg:ControlMode instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ControlMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-msg:mode-val is deprecated.  Use bluerov2_control-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlMode>)))
    "Constants for message type '<ControlMode>"
  '((:OFF . 0)
    (:IDLE . 1)
    (:ACCELTELEOP . 2)
    (:VELTELEOP . 3)
    (:HOLDPOSITION . 4)
    (:AUTOPILOT . 5)
    (:LOSGUIDANCE . 6)
    (:ABORT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlMode)))
    "Constants for message type 'ControlMode"
  '((:OFF . 0)
    (:IDLE . 1)
    (:ACCELTELEOP . 2)
    (:VELTELEOP . 3)
    (:HOLDPOSITION . 4)
    (:AUTOPILOT . 5)
    (:LOSGUIDANCE . 6)
    (:ABORT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMode>) ostream)
  "Serializes a message object of type '<ControlMode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMode>) istream)
  "Deserializes a message object of type '<ControlMode>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMode>)))
  "Returns string type for a message object of type '<ControlMode>"
  "bluerov2_control/ControlMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMode)))
  "Returns string type for a message object of type 'ControlMode"
  "bluerov2_control/ControlMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMode>)))
  "Returns md5sum for a message object of type '<ControlMode>"
  "c56dad23c66f87b189d2dc2a882cdf21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMode)))
  "Returns md5sum for a message object of type 'ControlMode"
  "c56dad23c66f87b189d2dc2a882cdf21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMode>)))
  "Returns full string definition for message of type '<ControlMode>"
  (cl:format cl:nil "uint8 OFF=0~%uint8 IDLE=1~%uint8 ACCELTELEOP=2~%uint8 VELTELEOP=3~%uint8 HOLDPOSITION=4~%uint8 AUTOPILOT=5~%uint8 LOSGUIDANCE=6~%uint8 ABORT=7~%~%uint8 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMode)))
  "Returns full string definition for message of type 'ControlMode"
  (cl:format cl:nil "uint8 OFF=0~%uint8 IDLE=1~%uint8 ACCELTELEOP=2~%uint8 VELTELEOP=3~%uint8 HOLDPOSITION=4~%uint8 AUTOPILOT=5~%uint8 LOSGUIDANCE=6~%uint8 ABORT=7~%~%uint8 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMode>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMode>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMode
    (cl:cons ':mode (mode msg))
))

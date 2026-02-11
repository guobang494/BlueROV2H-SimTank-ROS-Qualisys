; Auto-generated. Do not edit!


(cl:in-package bluerov2_control-srv)


;//! \htmlinclude SetControlMode-request.msg.html

(cl:defclass <SetControlMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type bluerov2_control-msg:ControlMode
    :initform (cl:make-instance 'bluerov2_control-msg:ControlMode)))
)

(cl:defclass SetControlMode-request (<SetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-srv:<SetControlMode-request> is deprecated: use bluerov2_control-srv:SetControlMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-srv:mode-val is deprecated.  Use bluerov2_control-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-request>) ostream)
  "Serializes a message object of type '<SetControlMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mode) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-request>) istream)
  "Deserializes a message object of type '<SetControlMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mode) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-request>)))
  "Returns string type for a service object of type '<SetControlMode-request>"
  "bluerov2_control/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-request)))
  "Returns string type for a service object of type 'SetControlMode-request"
  "bluerov2_control/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-request>)))
  "Returns md5sum for a message object of type '<SetControlMode-request>"
  "2b04d4d97f408d2de1f22ad0896570fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-request)))
  "Returns md5sum for a message object of type 'SetControlMode-request"
  "2b04d4d97f408d2de1f22ad0896570fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-request>)))
  "Returns full string definition for message of type '<SetControlMode-request>"
  (cl:format cl:nil "bluerov2_control/ControlMode mode~%~%================================================================================~%MSG: bluerov2_control/ControlMode~%uint8 OFF=0~%uint8 IDLE=1~%uint8 ACCELTELEOP=2~%uint8 VELTELEOP=3~%uint8 HOLDPOSITION=4~%uint8 AUTOPILOT=5~%uint8 LOSGUIDANCE=6~%uint8 ABORT=7~%~%uint8 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-request)))
  "Returns full string definition for message of type 'SetControlMode-request"
  (cl:format cl:nil "bluerov2_control/ControlMode mode~%~%================================================================================~%MSG: bluerov2_control/ControlMode~%uint8 OFF=0~%uint8 IDLE=1~%uint8 ACCELTELEOP=2~%uint8 VELTELEOP=3~%uint8 HOLDPOSITION=4~%uint8 AUTOPILOT=5~%uint8 LOSGUIDANCE=6~%uint8 ABORT=7~%~%uint8 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetControlMode-response.msg.html

(cl:defclass <SetControlMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetControlMode-response (<SetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-srv:<SetControlMode-response> is deprecated: use bluerov2_control-srv:SetControlMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-srv:success-val is deprecated.  Use bluerov2_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-response>) ostream)
  "Serializes a message object of type '<SetControlMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-response>) istream)
  "Deserializes a message object of type '<SetControlMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-response>)))
  "Returns string type for a service object of type '<SetControlMode-response>"
  "bluerov2_control/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-response)))
  "Returns string type for a service object of type 'SetControlMode-response"
  "bluerov2_control/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-response>)))
  "Returns md5sum for a message object of type '<SetControlMode-response>"
  "2b04d4d97f408d2de1f22ad0896570fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-response)))
  "Returns md5sum for a message object of type 'SetControlMode-response"
  "2b04d4d97f408d2de1f22ad0896570fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-response>)))
  "Returns full string definition for message of type '<SetControlMode-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-response)))
  "Returns full string definition for message of type 'SetControlMode-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode)))
  "Returns string type for a service object of type '<SetControlMode>"
  "bluerov2_control/SetControlMode")
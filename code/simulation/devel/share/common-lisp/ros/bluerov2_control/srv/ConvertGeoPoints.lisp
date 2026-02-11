; Auto-generated. Do not edit!


(cl:in-package bluerov2_control-srv)


;//! \htmlinclude ConvertGeoPoints-request.msg.html

(cl:defclass <ConvertGeoPoints-request> (roslisp-msg-protocol:ros-message)
  ((geopoints
    :reader geopoints
    :initarg :geopoints
    :type (cl:vector geographic_msgs-msg:GeoPoint)
   :initform (cl:make-array 0 :element-type 'geographic_msgs-msg:GeoPoint :initial-element (cl:make-instance 'geographic_msgs-msg:GeoPoint))))
)

(cl:defclass ConvertGeoPoints-request (<ConvertGeoPoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertGeoPoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertGeoPoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-srv:<ConvertGeoPoints-request> is deprecated: use bluerov2_control-srv:ConvertGeoPoints-request instead.")))

(cl:ensure-generic-function 'geopoints-val :lambda-list '(m))
(cl:defmethod geopoints-val ((m <ConvertGeoPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-srv:geopoints-val is deprecated.  Use bluerov2_control-srv:geopoints instead.")
  (geopoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertGeoPoints-request>) ostream)
  "Serializes a message object of type '<ConvertGeoPoints-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'geopoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'geopoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertGeoPoints-request>) istream)
  "Deserializes a message object of type '<ConvertGeoPoints-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'geopoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'geopoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geographic_msgs-msg:GeoPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertGeoPoints-request>)))
  "Returns string type for a service object of type '<ConvertGeoPoints-request>"
  "bluerov2_control/ConvertGeoPointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGeoPoints-request)))
  "Returns string type for a service object of type 'ConvertGeoPoints-request"
  "bluerov2_control/ConvertGeoPointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertGeoPoints-request>)))
  "Returns md5sum for a message object of type '<ConvertGeoPoints-request>"
  "a34bd81e20a7f59208cae9a3910e02a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertGeoPoints-request)))
  "Returns md5sum for a message object of type 'ConvertGeoPoints-request"
  "a34bd81e20a7f59208cae9a3910e02a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertGeoPoints-request>)))
  "Returns full string definition for message of type '<ConvertGeoPoints-request>"
  (cl:format cl:nil "geographic_msgs/GeoPoint[] geopoints~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertGeoPoints-request)))
  "Returns full string definition for message of type 'ConvertGeoPoints-request"
  (cl:format cl:nil "geographic_msgs/GeoPoint[] geopoints~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertGeoPoints-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'geopoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertGeoPoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertGeoPoints-request
    (cl:cons ':geopoints (geopoints msg))
))
;//! \htmlinclude ConvertGeoPoints-response.msg.html

(cl:defclass <ConvertGeoPoints-response> (roslisp-msg-protocol:ros-message)
  ((utmpoints
    :reader utmpoints
    :initarg :utmpoints
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ConvertGeoPoints-response (<ConvertGeoPoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertGeoPoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertGeoPoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluerov2_control-srv:<ConvertGeoPoints-response> is deprecated: use bluerov2_control-srv:ConvertGeoPoints-response instead.")))

(cl:ensure-generic-function 'utmpoints-val :lambda-list '(m))
(cl:defmethod utmpoints-val ((m <ConvertGeoPoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluerov2_control-srv:utmpoints-val is deprecated.  Use bluerov2_control-srv:utmpoints instead.")
  (utmpoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertGeoPoints-response>) ostream)
  "Serializes a message object of type '<ConvertGeoPoints-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'utmpoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'utmpoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertGeoPoints-response>) istream)
  "Deserializes a message object of type '<ConvertGeoPoints-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'utmpoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'utmpoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertGeoPoints-response>)))
  "Returns string type for a service object of type '<ConvertGeoPoints-response>"
  "bluerov2_control/ConvertGeoPointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGeoPoints-response)))
  "Returns string type for a service object of type 'ConvertGeoPoints-response"
  "bluerov2_control/ConvertGeoPointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertGeoPoints-response>)))
  "Returns md5sum for a message object of type '<ConvertGeoPoints-response>"
  "a34bd81e20a7f59208cae9a3910e02a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertGeoPoints-response)))
  "Returns md5sum for a message object of type 'ConvertGeoPoints-response"
  "a34bd81e20a7f59208cae9a3910e02a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertGeoPoints-response>)))
  "Returns full string definition for message of type '<ConvertGeoPoints-response>"
  (cl:format cl:nil "geometry_msgs/Point[] utmpoints~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertGeoPoints-response)))
  "Returns full string definition for message of type 'ConvertGeoPoints-response"
  (cl:format cl:nil "geometry_msgs/Point[] utmpoints~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertGeoPoints-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'utmpoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertGeoPoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertGeoPoints-response
    (cl:cons ':utmpoints (utmpoints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConvertGeoPoints)))
  'ConvertGeoPoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConvertGeoPoints)))
  'ConvertGeoPoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGeoPoints)))
  "Returns string type for a service object of type '<ConvertGeoPoints>"
  "bluerov2_control/ConvertGeoPoints")
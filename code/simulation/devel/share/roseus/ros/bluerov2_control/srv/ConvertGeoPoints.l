;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_control::ConvertGeoPoints)
  (if (not (find-package "BLUEROV2_CONTROL"))
    (make-package "BLUEROV2_CONTROL"))
  (shadow 'ConvertGeoPoints (find-package "BLUEROV2_CONTROL")))
(unless (find-package "BLUEROV2_CONTROL::CONVERTGEOPOINTS")
  (make-package "BLUEROV2_CONTROL::CONVERTGEOPOINTS"))
(unless (find-package "BLUEROV2_CONTROL::CONVERTGEOPOINTSREQUEST")
  (make-package "BLUEROV2_CONTROL::CONVERTGEOPOINTSREQUEST"))
(unless (find-package "BLUEROV2_CONTROL::CONVERTGEOPOINTSRESPONSE")
  (make-package "BLUEROV2_CONTROL::CONVERTGEOPOINTSRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOGRAPHIC_MSGS"))
  (ros::roseus-add-msgs "geographic_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass bluerov2_control::ConvertGeoPointsRequest
  :super ros::object
  :slots (_geopoints ))

(defmethod bluerov2_control::ConvertGeoPointsRequest
  (:init
   (&key
    ((:geopoints __geopoints) ())
    )
   (send-super :init)
   (setq _geopoints __geopoints)
   self)
  (:geopoints
   (&rest __geopoints)
   (if (keywordp (car __geopoints))
       (send* _geopoints __geopoints)
     (progn
       (if __geopoints (setq _geopoints (car __geopoints)))
       _geopoints)))
  (:serialization-length
   ()
   (+
    ;; geographic_msgs/GeoPoint[] _geopoints
    (apply #'+ (send-all _geopoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geographic_msgs/GeoPoint[] _geopoints
     (write-long (length _geopoints) s)
     (dolist (elem _geopoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geographic_msgs/GeoPoint[] _geopoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _geopoints (let (r) (dotimes (i n) (push (instance geographic_msgs::GeoPoint :init) r)) r))
     (dolist (elem- _geopoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass bluerov2_control::ConvertGeoPointsResponse
  :super ros::object
  :slots (_utmpoints ))

(defmethod bluerov2_control::ConvertGeoPointsResponse
  (:init
   (&key
    ((:utmpoints __utmpoints) ())
    )
   (send-super :init)
   (setq _utmpoints __utmpoints)
   self)
  (:utmpoints
   (&rest __utmpoints)
   (if (keywordp (car __utmpoints))
       (send* _utmpoints __utmpoints)
     (progn
       (if __utmpoints (setq _utmpoints (car __utmpoints)))
       _utmpoints)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _utmpoints
    (apply #'+ (send-all _utmpoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _utmpoints
     (write-long (length _utmpoints) s)
     (dolist (elem _utmpoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _utmpoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _utmpoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _utmpoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass bluerov2_control::ConvertGeoPoints
  :super ros::object
  :slots ())

(setf (get bluerov2_control::ConvertGeoPoints :md5sum-) "a34bd81e20a7f59208cae9a3910e02a9")
(setf (get bluerov2_control::ConvertGeoPoints :datatype-) "bluerov2_control/ConvertGeoPoints")
(setf (get bluerov2_control::ConvertGeoPoints :request) bluerov2_control::ConvertGeoPointsRequest)
(setf (get bluerov2_control::ConvertGeoPoints :response) bluerov2_control::ConvertGeoPointsResponse)

(defmethod bluerov2_control::ConvertGeoPointsRequest
  (:response () (instance bluerov2_control::ConvertGeoPointsResponse :init)))

(setf (get bluerov2_control::ConvertGeoPointsRequest :md5sum-) "a34bd81e20a7f59208cae9a3910e02a9")
(setf (get bluerov2_control::ConvertGeoPointsRequest :datatype-) "bluerov2_control/ConvertGeoPointsRequest")
(setf (get bluerov2_control::ConvertGeoPointsRequest :definition-)
      "geographic_msgs/GeoPoint[] geopoints

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude
---
geometry_msgs/Point[] utmpoints


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get bluerov2_control::ConvertGeoPointsResponse :md5sum-) "a34bd81e20a7f59208cae9a3910e02a9")
(setf (get bluerov2_control::ConvertGeoPointsResponse :datatype-) "bluerov2_control/ConvertGeoPointsResponse")
(setf (get bluerov2_control::ConvertGeoPointsResponse :definition-)
      "geographic_msgs/GeoPoint[] geopoints

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude
---
geometry_msgs/Point[] utmpoints


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :bluerov2_control/ConvertGeoPoints "a34bd81e20a7f59208cae9a3910e02a9")



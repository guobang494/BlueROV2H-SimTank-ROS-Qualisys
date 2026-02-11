
(cl:in-package :asdf)

(defsystem "bluerov2_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :bluerov2_control-msg
               :geographic_msgs-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ConvertGeoPoints" :depends-on ("_package_ConvertGeoPoints"))
    (:file "_package_ConvertGeoPoints" :depends-on ("_package"))
    (:file "SetControlMode" :depends-on ("_package_SetControlMode"))
    (:file "_package_SetControlMode" :depends-on ("_package"))
  ))
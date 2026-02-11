
(cl:in-package :asdf)

(defsystem "bluerov2_dobmpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Reference" :depends-on ("_package_Reference"))
    (:file "_package_Reference" :depends-on ("_package"))
  ))
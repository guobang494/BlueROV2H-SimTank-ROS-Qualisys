;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_control::SetControlMode)
  (if (not (find-package "BLUEROV2_CONTROL"))
    (make-package "BLUEROV2_CONTROL"))
  (shadow 'SetControlMode (find-package "BLUEROV2_CONTROL")))
(unless (find-package "BLUEROV2_CONTROL::SETCONTROLMODE")
  (make-package "BLUEROV2_CONTROL::SETCONTROLMODE"))
(unless (find-package "BLUEROV2_CONTROL::SETCONTROLMODEREQUEST")
  (make-package "BLUEROV2_CONTROL::SETCONTROLMODEREQUEST"))
(unless (find-package "BLUEROV2_CONTROL::SETCONTROLMODERESPONSE")
  (make-package "BLUEROV2_CONTROL::SETCONTROLMODERESPONSE"))

(in-package "ROS")





(defclass bluerov2_control::SetControlModeRequest
  :super ros::object
  :slots (_mode ))

(defmethod bluerov2_control::SetControlModeRequest
  (:init
   (&key
    ((:mode __mode) (instance bluerov2_control::ControlMode :init))
    )
   (send-super :init)
   (setq _mode __mode)
   self)
  (:mode
   (&rest __mode)
   (if (keywordp (car __mode))
       (send* _mode __mode)
     (progn
       (if __mode (setq _mode (car __mode)))
       _mode)))
  (:serialization-length
   ()
   (+
    ;; bluerov2_control/ControlMode _mode
    (send _mode :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bluerov2_control/ControlMode _mode
       (send _mode :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bluerov2_control/ControlMode _mode
     (send _mode :deserialize buf ptr-) (incf ptr- (send _mode :serialization-length))
   ;;
   self)
  )

(defclass bluerov2_control::SetControlModeResponse
  :super ros::object
  :slots (_success ))

(defmethod bluerov2_control::SetControlModeResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bluerov2_control::SetControlMode
  :super ros::object
  :slots ())

(setf (get bluerov2_control::SetControlMode :md5sum-) "2b04d4d97f408d2de1f22ad0896570fc")
(setf (get bluerov2_control::SetControlMode :datatype-) "bluerov2_control/SetControlMode")
(setf (get bluerov2_control::SetControlMode :request) bluerov2_control::SetControlModeRequest)
(setf (get bluerov2_control::SetControlMode :response) bluerov2_control::SetControlModeResponse)

(defmethod bluerov2_control::SetControlModeRequest
  (:response () (instance bluerov2_control::SetControlModeResponse :init)))

(setf (get bluerov2_control::SetControlModeRequest :md5sum-) "2b04d4d97f408d2de1f22ad0896570fc")
(setf (get bluerov2_control::SetControlModeRequest :datatype-) "bluerov2_control/SetControlModeRequest")
(setf (get bluerov2_control::SetControlModeRequest :definition-)
      "bluerov2_control/ControlMode mode

================================================================================
MSG: bluerov2_control/ControlMode
uint8 OFF=0
uint8 IDLE=1
uint8 ACCELTELEOP=2
uint8 VELTELEOP=3
uint8 HOLDPOSITION=4
uint8 AUTOPILOT=5
uint8 LOSGUIDANCE=6
uint8 ABORT=7

---
bool success

")

(setf (get bluerov2_control::SetControlModeResponse :md5sum-) "2b04d4d97f408d2de1f22ad0896570fc")
(setf (get bluerov2_control::SetControlModeResponse :datatype-) "bluerov2_control/SetControlModeResponse")
(setf (get bluerov2_control::SetControlModeResponse :definition-)
      "bluerov2_control/ControlMode mode

================================================================================
MSG: bluerov2_control/ControlMode
uint8 OFF=0
uint8 IDLE=1
uint8 ACCELTELEOP=2
uint8 VELTELEOP=3
uint8 HOLDPOSITION=4
uint8 AUTOPILOT=5
uint8 LOSGUIDANCE=6
uint8 ABORT=7

---
bool success

")



(provide :bluerov2_control/SetControlMode "2b04d4d97f408d2de1f22ad0896570fc")



;; Auto-generated. Do not edit!


(when (boundp 'bluerov2_control::ControlMode)
  (if (not (find-package "BLUEROV2_CONTROL"))
    (make-package "BLUEROV2_CONTROL"))
  (shadow 'ControlMode (find-package "BLUEROV2_CONTROL")))
(unless (find-package "BLUEROV2_CONTROL::CONTROLMODE")
  (make-package "BLUEROV2_CONTROL::CONTROLMODE"))

(in-package "ROS")
;;//! \htmlinclude ControlMode.msg.html


(intern "*OFF*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*OFF* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*OFF* 0)
(intern "*IDLE*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*IDLE* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*IDLE* 1)
(intern "*ACCELTELEOP*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*ACCELTELEOP* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*ACCELTELEOP* 2)
(intern "*VELTELEOP*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*VELTELEOP* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*VELTELEOP* 3)
(intern "*HOLDPOSITION*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*HOLDPOSITION* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*HOLDPOSITION* 4)
(intern "*AUTOPILOT*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*AUTOPILOT* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*AUTOPILOT* 5)
(intern "*LOSGUIDANCE*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*LOSGUIDANCE* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*LOSGUIDANCE* 6)
(intern "*ABORT*" (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(shadow '*ABORT* (find-package "BLUEROV2_CONTROL::CONTROLMODE"))
(defconstant bluerov2_control::ControlMode::*ABORT* 7)

(defun bluerov2_control::ControlMode-to-symbol (const)
  (cond
        ((= const 0) 'bluerov2_control::ControlMode::*OFF*)
        ((= const 1) 'bluerov2_control::ControlMode::*IDLE*)
        ((= const 2) 'bluerov2_control::ControlMode::*ACCELTELEOP*)
        ((= const 3) 'bluerov2_control::ControlMode::*VELTELEOP*)
        ((= const 4) 'bluerov2_control::ControlMode::*HOLDPOSITION*)
        ((= const 5) 'bluerov2_control::ControlMode::*AUTOPILOT*)
        ((= const 6) 'bluerov2_control::ControlMode::*LOSGUIDANCE*)
        ((= const 7) 'bluerov2_control::ControlMode::*ABORT*)
        (t nil)))

(defclass bluerov2_control::ControlMode
  :super ros::object
  :slots (_mode ))

(defmethod bluerov2_control::ControlMode
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get bluerov2_control::ControlMode :md5sum-) "c56dad23c66f87b189d2dc2a882cdf21")
(setf (get bluerov2_control::ControlMode :datatype-) "bluerov2_control/ControlMode")
(setf (get bluerov2_control::ControlMode :definition-)
      "uint8 OFF=0
uint8 IDLE=1
uint8 ACCELTELEOP=2
uint8 VELTELEOP=3
uint8 HOLDPOSITION=4
uint8 AUTOPILOT=5
uint8 LOSGUIDANCE=6
uint8 ABORT=7

uint8 mode
")



(provide :bluerov2_control/ControlMode "c56dad23c66f87b189d2dc2a882cdf21")



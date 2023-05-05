;; Auto-generated. Do not edit!


(when (boundp 'qrotor_firmware::Setpoint)
  (if (not (find-package "QROTOR_FIRMWARE"))
    (make-package "QROTOR_FIRMWARE"))
  (shadow 'Setpoint (find-package "QROTOR_FIRMWARE")))
(unless (find-package "QROTOR_FIRMWARE::SETPOINT")
  (make-package "QROTOR_FIRMWARE::SETPOINT"))
(unless (find-package "QROTOR_FIRMWARE::SETPOINTREQUEST")
  (make-package "QROTOR_FIRMWARE::SETPOINTREQUEST"))
(unless (find-package "QROTOR_FIRMWARE::SETPOINTRESPONSE")
  (make-package "QROTOR_FIRMWARE::SETPOINTRESPONSE"))

(in-package "ROS")





(defclass qrotor_firmware::SetpointRequest
  :super ros::object
  :slots (_x _y _z ))

(defmethod qrotor_firmware::SetpointRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass qrotor_firmware::SetpointResponse
  :super ros::object
  :slots (_success ))

(defmethod qrotor_firmware::SetpointResponse
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

(defclass qrotor_firmware::Setpoint
  :super ros::object
  :slots ())

(setf (get qrotor_firmware::Setpoint :md5sum-) "14fb54e9e518f55d418823395ca25d0b")
(setf (get qrotor_firmware::Setpoint :datatype-) "qrotor_firmware/Setpoint")
(setf (get qrotor_firmware::Setpoint :request) qrotor_firmware::SetpointRequest)
(setf (get qrotor_firmware::Setpoint :response) qrotor_firmware::SetpointResponse)

(defmethod qrotor_firmware::SetpointRequest
  (:response () (instance qrotor_firmware::SetpointResponse :init)))

(setf (get qrotor_firmware::SetpointRequest :md5sum-) "14fb54e9e518f55d418823395ca25d0b")
(setf (get qrotor_firmware::SetpointRequest :datatype-) "qrotor_firmware/SetpointRequest")
(setf (get qrotor_firmware::SetpointRequest :definition-)
      "float64 x
float64 y
float64 z
---
bool success
")

(setf (get qrotor_firmware::SetpointResponse :md5sum-) "14fb54e9e518f55d418823395ca25d0b")
(setf (get qrotor_firmware::SetpointResponse :datatype-) "qrotor_firmware/SetpointResponse")
(setf (get qrotor_firmware::SetpointResponse :definition-)
      "float64 x
float64 y
float64 z
---
bool success
")



(provide :qrotor_firmware/Setpoint "14fb54e9e518f55d418823395ca25d0b")



;; Auto-generated. Do not edit!


(when (boundp 'qrotor_firmware::FlatTrajectory)
  (if (not (find-package "QROTOR_FIRMWARE"))
    (make-package "QROTOR_FIRMWARE"))
  (shadow 'FlatTrajectory (find-package "QROTOR_FIRMWARE")))
(unless (find-package "QROTOR_FIRMWARE::FLATTRAJECTORY")
  (make-package "QROTOR_FIRMWARE::FLATTRAJECTORY"))
(unless (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST")
  (make-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(unless (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYRESPONSE")
  (make-package "QROTOR_FIRMWARE::FLATTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*TRAJECTORY_STRAIGHT_LINE*" (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(shadow '*TRAJECTORY_STRAIGHT_LINE* (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(defconstant qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_STRAIGHT_LINE* 0)
(intern "*TRAJECTORY_CIRCLE_2D*" (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(shadow '*TRAJECTORY_CIRCLE_2D* (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(defconstant qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_CIRCLE_2D* 1)
(intern "*TRAJECTORY_CIRCLE_3D*" (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(shadow '*TRAJECTORY_CIRCLE_3D* (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(defconstant qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_CIRCLE_3D* 2)
(intern "*TRAJECTORY_ELLIPSE_2D*" (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(shadow '*TRAJECTORY_ELLIPSE_2D* (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(defconstant qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_ELLIPSE_2D* 3)
(intern "*TRAJECTORY_ELLIPSE_3D*" (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(shadow '*TRAJECTORY_ELLIPSE_3D* (find-package "QROTOR_FIRMWARE::FLATTRAJECTORYREQUEST"))
(defconstant qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_ELLIPSE_3D* 4)

(defun qrotor_firmware::FlatTrajectoryRequest-to-symbol (const)
  (cond
        ((= const 0) 'qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_STRAIGHT_LINE*)
        ((= const 1) 'qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_CIRCLE_2D*)
        ((= const 2) 'qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_CIRCLE_3D*)
        ((= const 3) 'qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_ELLIPSE_2D*)
        ((= const 4) 'qrotor_firmware::FlatTrajectoryRequest::*TRAJECTORY_ELLIPSE_3D*)
        (t nil)))

(defclass qrotor_firmware::FlatTrajectoryRequest
  :super ros::object
  :slots (_type _center _radius _phase ))

(defmethod qrotor_firmware::FlatTrajectoryRequest
  (:init
   (&key
    ((:type __type) 0)
    ((:center __center) (instance geometry_msgs::Vector3 :init))
    ((:radius __radius) (instance geometry_msgs::Vector3 :init))
    ((:phase __phase) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _center __center)
   (setq _radius __radius)
   (setq _phase __phase)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:radius
   (&rest __radius)
   (if (keywordp (car __radius))
       (send* _radius __radius)
     (progn
       (if __radius (setq _radius (car __radius)))
       _radius)))
  (:phase
   (&rest __phase)
   (if (keywordp (car __phase))
       (send* _phase __phase)
     (progn
       (if __phase (setq _phase (car __phase)))
       _phase)))
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; geometry_msgs/Vector3 _center
    (send _center :serialization-length)
    ;; geometry_msgs/Vector3 _radius
    (send _radius :serialization-length)
    ;; geometry_msgs/Vector3 _phase
    (send _phase :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Vector3 _center
       (send _center :serialize s)
     ;; geometry_msgs/Vector3 _radius
       (send _radius :serialize s)
     ;; geometry_msgs/Vector3 _phase
       (send _phase :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; geometry_msgs/Vector3 _radius
     (send _radius :deserialize buf ptr-) (incf ptr- (send _radius :serialization-length))
   ;; geometry_msgs/Vector3 _phase
     (send _phase :deserialize buf ptr-) (incf ptr- (send _phase :serialization-length))
   ;;
   self)
  )

(defclass qrotor_firmware::FlatTrajectoryResponse
  :super ros::object
  :slots (_success ))

(defmethod qrotor_firmware::FlatTrajectoryResponse
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

(defclass qrotor_firmware::FlatTrajectory
  :super ros::object
  :slots ())

(setf (get qrotor_firmware::FlatTrajectory :md5sum-) "df0bac3bd179de8433b17a584a39b688")
(setf (get qrotor_firmware::FlatTrajectory :datatype-) "qrotor_firmware/FlatTrajectory")
(setf (get qrotor_firmware::FlatTrajectory :request) qrotor_firmware::FlatTrajectoryRequest)
(setf (get qrotor_firmware::FlatTrajectory :response) qrotor_firmware::FlatTrajectoryResponse)

(defmethod qrotor_firmware::FlatTrajectoryRequest
  (:response () (instance qrotor_firmware::FlatTrajectoryResponse :init)))

(setf (get qrotor_firmware::FlatTrajectoryRequest :md5sum-) "df0bac3bd179de8433b17a584a39b688")
(setf (get qrotor_firmware::FlatTrajectoryRequest :datatype-) "qrotor_firmware/FlatTrajectoryRequest")
(setf (get qrotor_firmware::FlatTrajectoryRequest :definition-)
      "# set Trajectory type
uint8 TRAJECTORY_STRAIGHT_LINE = 0
uint8 TRAJECTORY_CIRCLE_2D = 1
uint8 TRAJECTORY_CIRCLE_3D = 2
uint8 TRAJECTORY_ELLIPSE_2D = 3
uint8 TRAJECTORY_ELLIPSE_3D = 4

uint8 type
geometry_msgs/Vector3 center
geometry_msgs/Vector3 radius
geometry_msgs/Vector3 phase

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
")

(setf (get qrotor_firmware::FlatTrajectoryResponse :md5sum-) "df0bac3bd179de8433b17a584a39b688")
(setf (get qrotor_firmware::FlatTrajectoryResponse :datatype-) "qrotor_firmware/FlatTrajectoryResponse")
(setf (get qrotor_firmware::FlatTrajectoryResponse :definition-)
      "# set Trajectory type
uint8 TRAJECTORY_STRAIGHT_LINE = 0
uint8 TRAJECTORY_CIRCLE_2D = 1
uint8 TRAJECTORY_CIRCLE_3D = 2
uint8 TRAJECTORY_ELLIPSE_2D = 3
uint8 TRAJECTORY_ELLIPSE_3D = 4

uint8 type
geometry_msgs/Vector3 center
geometry_msgs/Vector3 radius
geometry_msgs/Vector3 phase

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
")



(provide :qrotor_firmware/FlatTrajectory "df0bac3bd179de8433b17a584a39b688")



;; Auto-generated. Do not edit!


(when (boundp 'qrotor_firmware::AttitudeState)
  (if (not (find-package "QROTOR_FIRMWARE"))
    (make-package "QROTOR_FIRMWARE"))
  (shadow 'AttitudeState (find-package "QROTOR_FIRMWARE")))
(unless (find-package "QROTOR_FIRMWARE::ATTITUDESTATE")
  (make-package "QROTOR_FIRMWARE::ATTITUDESTATE"))

(in-package "ROS")
;;//! \htmlinclude AttitudeState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qrotor_firmware::AttitudeState
  :super ros::object
  :slots (_header _quaternion _body_rates _angular_acceleration ))

(defmethod qrotor_firmware::AttitudeState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quaternion __quaternion) (instance geometry_msgs::Quaternion :init))
    ((:body_rates __body_rates) (instance geometry_msgs::Vector3 :init))
    ((:angular_acceleration __angular_acceleration) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _quaternion __quaternion)
   (setq _body_rates __body_rates)
   (setq _angular_acceleration __angular_acceleration)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quaternion
   (&rest __quaternion)
   (if (keywordp (car __quaternion))
       (send* _quaternion __quaternion)
     (progn
       (if __quaternion (setq _quaternion (car __quaternion)))
       _quaternion)))
  (:body_rates
   (&rest __body_rates)
   (if (keywordp (car __body_rates))
       (send* _body_rates __body_rates)
     (progn
       (if __body_rates (setq _body_rates (car __body_rates)))
       _body_rates)))
  (:angular_acceleration
   (&rest __angular_acceleration)
   (if (keywordp (car __angular_acceleration))
       (send* _angular_acceleration __angular_acceleration)
     (progn
       (if __angular_acceleration (setq _angular_acceleration (car __angular_acceleration)))
       _angular_acceleration)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _quaternion
    (send _quaternion :serialization-length)
    ;; geometry_msgs/Vector3 _body_rates
    (send _body_rates :serialization-length)
    ;; geometry_msgs/Vector3 _angular_acceleration
    (send _angular_acceleration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _quaternion
       (send _quaternion :serialize s)
     ;; geometry_msgs/Vector3 _body_rates
       (send _body_rates :serialize s)
     ;; geometry_msgs/Vector3 _angular_acceleration
       (send _angular_acceleration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _quaternion
     (send _quaternion :deserialize buf ptr-) (incf ptr- (send _quaternion :serialization-length))
   ;; geometry_msgs/Vector3 _body_rates
     (send _body_rates :deserialize buf ptr-) (incf ptr- (send _body_rates :serialization-length))
   ;; geometry_msgs/Vector3 _angular_acceleration
     (send _angular_acceleration :deserialize buf ptr-) (incf ptr- (send _angular_acceleration :serialization-length))
   ;;
   self)
  )

(setf (get qrotor_firmware::AttitudeState :md5sum-) "9948c65f79422ced49892ff33a8d7940")
(setf (get qrotor_firmware::AttitudeState :datatype-) "qrotor_firmware/AttitudeState")
(setf (get qrotor_firmware::AttitudeState :definition-)
      "# AttitudeState msg
std_msgs/Header             header
geometry_msgs/Quaternion    quaternion
geometry_msgs/Vector3       body_rates
geometry_msgs/Vector3       angular_acceleration
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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
float64 z
")



(provide :qrotor_firmware/AttitudeState "9948c65f79422ced49892ff33a8d7940")



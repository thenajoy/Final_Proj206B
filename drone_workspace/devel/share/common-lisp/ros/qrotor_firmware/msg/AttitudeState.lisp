; Auto-generated. Do not edit!


(cl:in-package qrotor_firmware-msg)


;//! \htmlinclude AttitudeState.msg.html

(cl:defclass <AttitudeState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (quaternion
    :reader quaternion
    :initarg :quaternion
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (body_rates
    :reader body_rates
    :initarg :body_rates
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_acceleration
    :reader angular_acceleration
    :initarg :angular_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass AttitudeState (<AttitudeState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttitudeState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttitudeState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qrotor_firmware-msg:<AttitudeState> is deprecated: use qrotor_firmware-msg:AttitudeState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttitudeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-msg:header-val is deprecated.  Use qrotor_firmware-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'quaternion-val :lambda-list '(m))
(cl:defmethod quaternion-val ((m <AttitudeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-msg:quaternion-val is deprecated.  Use qrotor_firmware-msg:quaternion instead.")
  (quaternion m))

(cl:ensure-generic-function 'body_rates-val :lambda-list '(m))
(cl:defmethod body_rates-val ((m <AttitudeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-msg:body_rates-val is deprecated.  Use qrotor_firmware-msg:body_rates instead.")
  (body_rates m))

(cl:ensure-generic-function 'angular_acceleration-val :lambda-list '(m))
(cl:defmethod angular_acceleration-val ((m <AttitudeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-msg:angular_acceleration-val is deprecated.  Use qrotor_firmware-msg:angular_acceleration instead.")
  (angular_acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttitudeState>) ostream)
  "Serializes a message object of type '<AttitudeState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quaternion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'body_rates) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_acceleration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttitudeState>) istream)
  "Deserializes a message object of type '<AttitudeState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quaternion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'body_rates) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_acceleration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttitudeState>)))
  "Returns string type for a message object of type '<AttitudeState>"
  "qrotor_firmware/AttitudeState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttitudeState)))
  "Returns string type for a message object of type 'AttitudeState"
  "qrotor_firmware/AttitudeState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttitudeState>)))
  "Returns md5sum for a message object of type '<AttitudeState>"
  "9948c65f79422ced49892ff33a8d7940")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttitudeState)))
  "Returns md5sum for a message object of type 'AttitudeState"
  "9948c65f79422ced49892ff33a8d7940")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttitudeState>)))
  "Returns full string definition for message of type '<AttitudeState>"
  (cl:format cl:nil "# AttitudeState msg~%std_msgs/Header             header~%geometry_msgs/Quaternion    quaternion~%geometry_msgs/Vector3       body_rates~%geometry_msgs/Vector3       angular_acceleration~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttitudeState)))
  "Returns full string definition for message of type 'AttitudeState"
  (cl:format cl:nil "# AttitudeState msg~%std_msgs/Header             header~%geometry_msgs/Quaternion    quaternion~%geometry_msgs/Vector3       body_rates~%geometry_msgs/Vector3       angular_acceleration~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttitudeState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quaternion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'body_rates))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_acceleration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttitudeState>))
  "Converts a ROS message object to a list"
  (cl:list 'AttitudeState
    (cl:cons ':header (header msg))
    (cl:cons ':quaternion (quaternion msg))
    (cl:cons ':body_rates (body_rates msg))
    (cl:cons ':angular_acceleration (angular_acceleration msg))
))

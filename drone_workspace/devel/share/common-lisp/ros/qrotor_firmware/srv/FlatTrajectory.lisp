; Auto-generated. Do not edit!


(cl:in-package qrotor_firmware-srv)


;//! \htmlinclude FlatTrajectory-request.msg.html

(cl:defclass <FlatTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (radius
    :reader radius
    :initarg :radius
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (phase
    :reader phase
    :initarg :phase
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass FlatTrajectory-request (<FlatTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlatTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlatTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qrotor_firmware-srv:<FlatTrajectory-request> is deprecated: use qrotor_firmware-srv:FlatTrajectory-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <FlatTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:type-val is deprecated.  Use qrotor_firmware-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <FlatTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:center-val is deprecated.  Use qrotor_firmware-srv:center instead.")
  (center m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <FlatTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:radius-val is deprecated.  Use qrotor_firmware-srv:radius instead.")
  (radius m))

(cl:ensure-generic-function 'phase-val :lambda-list '(m))
(cl:defmethod phase-val ((m <FlatTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:phase-val is deprecated.  Use qrotor_firmware-srv:phase instead.")
  (phase m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlatTrajectory-request>)))
    "Constants for message type '<FlatTrajectory-request>"
  '((:TRAJECTORY_STRAIGHT_LINE . 0)
    (:TRAJECTORY_CIRCLE_2D . 1)
    (:TRAJECTORY_CIRCLE_3D . 2)
    (:TRAJECTORY_ELLIPSE_2D . 3)
    (:TRAJECTORY_ELLIPSE_3D . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlatTrajectory-request)))
    "Constants for message type 'FlatTrajectory-request"
  '((:TRAJECTORY_STRAIGHT_LINE . 0)
    (:TRAJECTORY_CIRCLE_2D . 1)
    (:TRAJECTORY_CIRCLE_3D . 2)
    (:TRAJECTORY_ELLIPSE_2D . 3)
    (:TRAJECTORY_ELLIPSE_3D . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlatTrajectory-request>) ostream)
  "Serializes a message object of type '<FlatTrajectory-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'radius) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'phase) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlatTrajectory-request>) istream)
  "Deserializes a message object of type '<FlatTrajectory-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'radius) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'phase) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlatTrajectory-request>)))
  "Returns string type for a service object of type '<FlatTrajectory-request>"
  "qrotor_firmware/FlatTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlatTrajectory-request)))
  "Returns string type for a service object of type 'FlatTrajectory-request"
  "qrotor_firmware/FlatTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlatTrajectory-request>)))
  "Returns md5sum for a message object of type '<FlatTrajectory-request>"
  "df0bac3bd179de8433b17a584a39b688")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlatTrajectory-request)))
  "Returns md5sum for a message object of type 'FlatTrajectory-request"
  "df0bac3bd179de8433b17a584a39b688")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlatTrajectory-request>)))
  "Returns full string definition for message of type '<FlatTrajectory-request>"
  (cl:format cl:nil "# set Trajectory type~%uint8 TRAJECTORY_STRAIGHT_LINE = 0~%uint8 TRAJECTORY_CIRCLE_2D = 1~%uint8 TRAJECTORY_CIRCLE_3D = 2~%uint8 TRAJECTORY_ELLIPSE_2D = 3~%uint8 TRAJECTORY_ELLIPSE_3D = 4~%~%uint8 type~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 radius~%geometry_msgs/Vector3 phase~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlatTrajectory-request)))
  "Returns full string definition for message of type 'FlatTrajectory-request"
  (cl:format cl:nil "# set Trajectory type~%uint8 TRAJECTORY_STRAIGHT_LINE = 0~%uint8 TRAJECTORY_CIRCLE_2D = 1~%uint8 TRAJECTORY_CIRCLE_3D = 2~%uint8 TRAJECTORY_ELLIPSE_2D = 3~%uint8 TRAJECTORY_ELLIPSE_3D = 4~%~%uint8 type~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 radius~%geometry_msgs/Vector3 phase~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlatTrajectory-request>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'radius))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'phase))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlatTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FlatTrajectory-request
    (cl:cons ':type (type msg))
    (cl:cons ':center (center msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':phase (phase msg))
))
;//! \htmlinclude FlatTrajectory-response.msg.html

(cl:defclass <FlatTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FlatTrajectory-response (<FlatTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlatTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlatTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qrotor_firmware-srv:<FlatTrajectory-response> is deprecated: use qrotor_firmware-srv:FlatTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <FlatTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:success-val is deprecated.  Use qrotor_firmware-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlatTrajectory-response>) ostream)
  "Serializes a message object of type '<FlatTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlatTrajectory-response>) istream)
  "Deserializes a message object of type '<FlatTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlatTrajectory-response>)))
  "Returns string type for a service object of type '<FlatTrajectory-response>"
  "qrotor_firmware/FlatTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlatTrajectory-response)))
  "Returns string type for a service object of type 'FlatTrajectory-response"
  "qrotor_firmware/FlatTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlatTrajectory-response>)))
  "Returns md5sum for a message object of type '<FlatTrajectory-response>"
  "df0bac3bd179de8433b17a584a39b688")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlatTrajectory-response)))
  "Returns md5sum for a message object of type 'FlatTrajectory-response"
  "df0bac3bd179de8433b17a584a39b688")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlatTrajectory-response>)))
  "Returns full string definition for message of type '<FlatTrajectory-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlatTrajectory-response)))
  "Returns full string definition for message of type 'FlatTrajectory-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlatTrajectory-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlatTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FlatTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FlatTrajectory)))
  'FlatTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FlatTrajectory)))
  'FlatTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlatTrajectory)))
  "Returns string type for a service object of type '<FlatTrajectory>"
  "qrotor_firmware/FlatTrajectory")
; Auto-generated. Do not edit!


(cl:in-package qrotor_firmware-srv)


;//! \htmlinclude Setpoint-request.msg.html

(cl:defclass <Setpoint-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Setpoint-request (<Setpoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Setpoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Setpoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qrotor_firmware-srv:<Setpoint-request> is deprecated: use qrotor_firmware-srv:Setpoint-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Setpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:x-val is deprecated.  Use qrotor_firmware-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Setpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:y-val is deprecated.  Use qrotor_firmware-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Setpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:z-val is deprecated.  Use qrotor_firmware-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Setpoint-request>) ostream)
  "Serializes a message object of type '<Setpoint-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Setpoint-request>) istream)
  "Deserializes a message object of type '<Setpoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Setpoint-request>)))
  "Returns string type for a service object of type '<Setpoint-request>"
  "qrotor_firmware/SetpointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Setpoint-request)))
  "Returns string type for a service object of type 'Setpoint-request"
  "qrotor_firmware/SetpointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Setpoint-request>)))
  "Returns md5sum for a message object of type '<Setpoint-request>"
  "14fb54e9e518f55d418823395ca25d0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Setpoint-request)))
  "Returns md5sum for a message object of type 'Setpoint-request"
  "14fb54e9e518f55d418823395ca25d0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Setpoint-request>)))
  "Returns full string definition for message of type '<Setpoint-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Setpoint-request)))
  "Returns full string definition for message of type 'Setpoint-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Setpoint-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Setpoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Setpoint-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude Setpoint-response.msg.html

(cl:defclass <Setpoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Setpoint-response (<Setpoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Setpoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Setpoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qrotor_firmware-srv:<Setpoint-response> is deprecated: use qrotor_firmware-srv:Setpoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Setpoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qrotor_firmware-srv:success-val is deprecated.  Use qrotor_firmware-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Setpoint-response>) ostream)
  "Serializes a message object of type '<Setpoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Setpoint-response>) istream)
  "Deserializes a message object of type '<Setpoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Setpoint-response>)))
  "Returns string type for a service object of type '<Setpoint-response>"
  "qrotor_firmware/SetpointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Setpoint-response)))
  "Returns string type for a service object of type 'Setpoint-response"
  "qrotor_firmware/SetpointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Setpoint-response>)))
  "Returns md5sum for a message object of type '<Setpoint-response>"
  "14fb54e9e518f55d418823395ca25d0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Setpoint-response)))
  "Returns md5sum for a message object of type 'Setpoint-response"
  "14fb54e9e518f55d418823395ca25d0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Setpoint-response>)))
  "Returns full string definition for message of type '<Setpoint-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Setpoint-response)))
  "Returns full string definition for message of type 'Setpoint-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Setpoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Setpoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Setpoint-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Setpoint)))
  'Setpoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Setpoint)))
  'Setpoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Setpoint)))
  "Returns string type for a service object of type '<Setpoint>"
  "qrotor_firmware/Setpoint")
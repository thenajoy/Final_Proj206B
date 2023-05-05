;; Auto-generated. Do not edit!


(when (boundp 'qrotor_firmware::Log)
  (if (not (find-package "QROTOR_FIRMWARE"))
    (make-package "QROTOR_FIRMWARE"))
  (shadow 'Log (find-package "QROTOR_FIRMWARE")))
(unless (find-package "QROTOR_FIRMWARE::LOG")
  (make-package "QROTOR_FIRMWARE::LOG"))

(in-package "ROS")
;;//! \htmlinclude Log.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qrotor_firmware::Log
  :super ros::object
  :slots (_header _euler _body_rates _linear_acceleration _angular_acceleration _cmd_euler _thrust _moment _position _velocity _loop_rate _voltage _current _esc_in_us _motors_state _control_mode _firmware_time _attitude_mode _lyapunov ))

(defmethod qrotor_firmware::Log
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:euler __euler) (instance geometry_msgs::Vector3 :init))
    ((:body_rates __body_rates) (instance geometry_msgs::Vector3 :init))
    ((:linear_acceleration __linear_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:angular_acceleration __angular_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:cmd_euler __cmd_euler) (instance geometry_msgs::Vector3 :init))
    ((:thrust __thrust) 0.0)
    ((:moment __moment) (instance geometry_msgs::Vector3 :init))
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:loop_rate __loop_rate) 0.0)
    ((:voltage __voltage) 0.0)
    ((:current __current) 0.0)
    ((:esc_in_us __esc_in_us) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:motors_state __motors_state) nil)
    ((:control_mode __control_mode) 0)
    ((:firmware_time __firmware_time) 0)
    ((:attitude_mode __attitude_mode) 0)
    ((:lyapunov __lyapunov) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _euler __euler)
   (setq _body_rates __body_rates)
   (setq _linear_acceleration __linear_acceleration)
   (setq _angular_acceleration __angular_acceleration)
   (setq _cmd_euler __cmd_euler)
   (setq _thrust (float __thrust))
   (setq _moment __moment)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _loop_rate (float __loop_rate))
   (setq _voltage (float __voltage))
   (setq _current (float __current))
   (setq _esc_in_us __esc_in_us)
   (setq _motors_state __motors_state)
   (setq _control_mode (round __control_mode))
   (setq _firmware_time (round __firmware_time))
   (setq _attitude_mode (round __attitude_mode))
   (setq _lyapunov (float __lyapunov))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:euler
   (&rest __euler)
   (if (keywordp (car __euler))
       (send* _euler __euler)
     (progn
       (if __euler (setq _euler (car __euler)))
       _euler)))
  (:body_rates
   (&rest __body_rates)
   (if (keywordp (car __body_rates))
       (send* _body_rates __body_rates)
     (progn
       (if __body_rates (setq _body_rates (car __body_rates)))
       _body_rates)))
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:angular_acceleration
   (&rest __angular_acceleration)
   (if (keywordp (car __angular_acceleration))
       (send* _angular_acceleration __angular_acceleration)
     (progn
       (if __angular_acceleration (setq _angular_acceleration (car __angular_acceleration)))
       _angular_acceleration)))
  (:cmd_euler
   (&rest __cmd_euler)
   (if (keywordp (car __cmd_euler))
       (send* _cmd_euler __cmd_euler)
     (progn
       (if __cmd_euler (setq _cmd_euler (car __cmd_euler)))
       _cmd_euler)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:moment
   (&rest __moment)
   (if (keywordp (car __moment))
       (send* _moment __moment)
     (progn
       (if __moment (setq _moment (car __moment)))
       _moment)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:loop_rate
   (&optional __loop_rate)
   (if __loop_rate (setq _loop_rate __loop_rate)) _loop_rate)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:esc_in_us
   (&optional __esc_in_us)
   (if __esc_in_us (setq _esc_in_us __esc_in_us)) _esc_in_us)
  (:motors_state
   (&optional (__motors_state :null))
   (if (not (eq __motors_state :null)) (setq _motors_state __motors_state)) _motors_state)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:firmware_time
   (&optional __firmware_time)
   (if __firmware_time (setq _firmware_time __firmware_time)) _firmware_time)
  (:attitude_mode
   (&optional __attitude_mode)
   (if __attitude_mode (setq _attitude_mode __attitude_mode)) _attitude_mode)
  (:lyapunov
   (&optional __lyapunov)
   (if __lyapunov (setq _lyapunov __lyapunov)) _lyapunov)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _euler
    (send _euler :serialization-length)
    ;; geometry_msgs/Vector3 _body_rates
    (send _body_rates :serialization-length)
    ;; geometry_msgs/Vector3 _linear_acceleration
    (send _linear_acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _angular_acceleration
    (send _angular_acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _cmd_euler
    (send _cmd_euler :serialization-length)
    ;; float64 _thrust
    8
    ;; geometry_msgs/Vector3 _moment
    (send _moment :serialization-length)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; float64 _loop_rate
    8
    ;; float64 _voltage
    8
    ;; float64 _current
    8
    ;; float64[4] _esc_in_us
    (* 8    4)
    ;; bool _motors_state
    1
    ;; int32 _control_mode
    4
    ;; uint64 _firmware_time
    8
    ;; uint8 _attitude_mode
    1
    ;; float64 _lyapunov
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _euler
       (send _euler :serialize s)
     ;; geometry_msgs/Vector3 _body_rates
       (send _body_rates :serialize s)
     ;; geometry_msgs/Vector3 _linear_acceleration
       (send _linear_acceleration :serialize s)
     ;; geometry_msgs/Vector3 _angular_acceleration
       (send _angular_acceleration :serialize s)
     ;; geometry_msgs/Vector3 _cmd_euler
       (send _cmd_euler :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _moment
       (send _moment :serialize s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; float64 _loop_rate
       (sys::poke _loop_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[4] _esc_in_us
     (dotimes (i 4)
       (sys::poke (elt _esc_in_us i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _motors_state
       (if _motors_state (write-byte -1 s) (write-byte 0 s))
     ;; int32 _control_mode
       (write-long _control_mode s)
     ;; uint64 _firmware_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _firmware_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _firmware_time) (= (length (_firmware_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_firmware_time . bv) 0) 0) s)
              (write-long (ash (elt (_firmware_time . bv) 1) -1) s))
             ((and (class _firmware_time) (= (length (_firmware_time . bv)) 1)) ;; big1
              (write-long (elt (_firmware_time . bv) 0) s)
              (write-long (if (>= _firmware_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _firmware_time s)(write-long (if (>= _firmware_time 0) 0 #xffffffff) s)))
     ;; uint8 _attitude_mode
       (write-byte _attitude_mode s)
     ;; float64 _lyapunov
       (sys::poke _lyapunov (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _euler
     (send _euler :deserialize buf ptr-) (incf ptr- (send _euler :serialization-length))
   ;; geometry_msgs/Vector3 _body_rates
     (send _body_rates :deserialize buf ptr-) (incf ptr- (send _body_rates :serialization-length))
   ;; geometry_msgs/Vector3 _linear_acceleration
     (send _linear_acceleration :deserialize buf ptr-) (incf ptr- (send _linear_acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _angular_acceleration
     (send _angular_acceleration :deserialize buf ptr-) (incf ptr- (send _angular_acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _cmd_euler
     (send _cmd_euler :deserialize buf ptr-) (incf ptr- (send _cmd_euler :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _moment
     (send _moment :deserialize buf ptr-) (incf ptr- (send _moment :serialization-length))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float64 _loop_rate
     (setq _loop_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[4] _esc_in_us
   (dotimes (i (length _esc_in_us))
     (setf (elt _esc_in_us i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _motors_state
     (setq _motors_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint64 _firmware_time
     
#+(or :alpha :irix6 :x86_64)
      (setf _firmware_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _firmware_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint8 _attitude_mode
     (setq _attitude_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _lyapunov
     (setq _lyapunov (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get qrotor_firmware::Log :md5sum-) "479e80fd954016432b0609f599bc1f43")
(setf (get qrotor_firmware::Log :datatype-) "qrotor_firmware/Log")
(setf (get qrotor_firmware::Log :definition-)
      "# Quadrotor Onboard Log 
std_msgs/Header         header
geometry_msgs/Vector3   euler
geometry_msgs/Vector3   body_rates
geometry_msgs/Vector3   linear_acceleration
geometry_msgs/Vector3   angular_acceleration
geometry_msgs/Vector3   cmd_euler
float64                 thrust
geometry_msgs/Vector3   moment
geometry_msgs/Vector3   position
geometry_msgs/Vector3   velocity
float64                 loop_rate
float64                 voltage
float64                 current
float64[4]              esc_in_us
bool                    motors_state
int32                   control_mode
uint64                  firmware_time
uint8                   attitude_mode
float64                 lyapunov
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



(provide :qrotor_firmware/Log "479e80fd954016432b0609f599bc1f43")



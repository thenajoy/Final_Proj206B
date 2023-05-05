# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from qrotor_firmware/Log.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class Log(genpy.Message):
  _md5sum = "479e80fd954016432b0609f599bc1f43"
  _type = "qrotor_firmware/Log"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# Quadrotor Onboard Log 
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
float64 z"""
  __slots__ = ['header','euler','body_rates','linear_acceleration','angular_acceleration','cmd_euler','thrust','moment','position','velocity','loop_rate','voltage','current','esc_in_us','motors_state','control_mode','firmware_time','attitude_mode','lyapunov']
  _slot_types = ['std_msgs/Header','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','float64','float64','float64[4]','bool','int32','uint64','uint8','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,euler,body_rates,linear_acceleration,angular_acceleration,cmd_euler,thrust,moment,position,velocity,loop_rate,voltage,current,esc_in_us,motors_state,control_mode,firmware_time,attitude_mode,lyapunov

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Log, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.euler is None:
        self.euler = geometry_msgs.msg.Vector3()
      if self.body_rates is None:
        self.body_rates = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.cmd_euler is None:
        self.cmd_euler = geometry_msgs.msg.Vector3()
      if self.thrust is None:
        self.thrust = 0.
      if self.moment is None:
        self.moment = geometry_msgs.msg.Vector3()
      if self.position is None:
        self.position = geometry_msgs.msg.Vector3()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Vector3()
      if self.loop_rate is None:
        self.loop_rate = 0.
      if self.voltage is None:
        self.voltage = 0.
      if self.current is None:
        self.current = 0.
      if self.esc_in_us is None:
        self.esc_in_us = [0.] * 4
      if self.motors_state is None:
        self.motors_state = False
      if self.control_mode is None:
        self.control_mode = 0
      if self.firmware_time is None:
        self.firmware_time = 0
      if self.attitude_mode is None:
        self.attitude_mode = 0
      if self.lyapunov is None:
        self.lyapunov = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.euler = geometry_msgs.msg.Vector3()
      self.body_rates = geometry_msgs.msg.Vector3()
      self.linear_acceleration = geometry_msgs.msg.Vector3()
      self.angular_acceleration = geometry_msgs.msg.Vector3()
      self.cmd_euler = geometry_msgs.msg.Vector3()
      self.thrust = 0.
      self.moment = geometry_msgs.msg.Vector3()
      self.position = geometry_msgs.msg.Vector3()
      self.velocity = geometry_msgs.msg.Vector3()
      self.loop_rate = 0.
      self.voltage = 0.
      self.current = 0.
      self.esc_in_us = [0.] * 4
      self.motors_state = False
      self.control_mode = 0
      self.firmware_time = 0
      self.attitude_mode = 0
      self.lyapunov = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_28d().pack(_x.euler.x, _x.euler.y, _x.euler.z, _x.body_rates.x, _x.body_rates.y, _x.body_rates.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.cmd_euler.x, _x.cmd_euler.y, _x.cmd_euler.z, _x.thrust, _x.moment.x, _x.moment.y, _x.moment.z, _x.position.x, _x.position.y, _x.position.z, _x.velocity.x, _x.velocity.y, _x.velocity.z, _x.loop_rate, _x.voltage, _x.current))
      buff.write(_get_struct_4d().pack(*self.esc_in_us))
      _x = self
      buff.write(_get_struct_BiQBd().pack(_x.motors_state, _x.control_mode, _x.firmware_time, _x.attitude_mode, _x.lyapunov))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.euler is None:
        self.euler = geometry_msgs.msg.Vector3()
      if self.body_rates is None:
        self.body_rates = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.cmd_euler is None:
        self.cmd_euler = geometry_msgs.msg.Vector3()
      if self.moment is None:
        self.moment = geometry_msgs.msg.Vector3()
      if self.position is None:
        self.position = geometry_msgs.msg.Vector3()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 224
      (_x.euler.x, _x.euler.y, _x.euler.z, _x.body_rates.x, _x.body_rates.y, _x.body_rates.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.cmd_euler.x, _x.cmd_euler.y, _x.cmd_euler.z, _x.thrust, _x.moment.x, _x.moment.y, _x.moment.z, _x.position.x, _x.position.y, _x.position.z, _x.velocity.x, _x.velocity.y, _x.velocity.z, _x.loop_rate, _x.voltage, _x.current,) = _get_struct_28d().unpack(str[start:end])
      start = end
      end += 32
      self.esc_in_us = _get_struct_4d().unpack(str[start:end])
      _x = self
      start = end
      end += 22
      (_x.motors_state, _x.control_mode, _x.firmware_time, _x.attitude_mode, _x.lyapunov,) = _get_struct_BiQBd().unpack(str[start:end])
      self.motors_state = bool(self.motors_state)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_28d().pack(_x.euler.x, _x.euler.y, _x.euler.z, _x.body_rates.x, _x.body_rates.y, _x.body_rates.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.cmd_euler.x, _x.cmd_euler.y, _x.cmd_euler.z, _x.thrust, _x.moment.x, _x.moment.y, _x.moment.z, _x.position.x, _x.position.y, _x.position.z, _x.velocity.x, _x.velocity.y, _x.velocity.z, _x.loop_rate, _x.voltage, _x.current))
      buff.write(self.esc_in_us.tostring())
      _x = self
      buff.write(_get_struct_BiQBd().pack(_x.motors_state, _x.control_mode, _x.firmware_time, _x.attitude_mode, _x.lyapunov))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.euler is None:
        self.euler = geometry_msgs.msg.Vector3()
      if self.body_rates is None:
        self.body_rates = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.cmd_euler is None:
        self.cmd_euler = geometry_msgs.msg.Vector3()
      if self.moment is None:
        self.moment = geometry_msgs.msg.Vector3()
      if self.position is None:
        self.position = geometry_msgs.msg.Vector3()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 224
      (_x.euler.x, _x.euler.y, _x.euler.z, _x.body_rates.x, _x.body_rates.y, _x.body_rates.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.cmd_euler.x, _x.cmd_euler.y, _x.cmd_euler.z, _x.thrust, _x.moment.x, _x.moment.y, _x.moment.z, _x.position.x, _x.position.y, _x.position.z, _x.velocity.x, _x.velocity.y, _x.velocity.z, _x.loop_rate, _x.voltage, _x.current,) = _get_struct_28d().unpack(str[start:end])
      start = end
      end += 32
      self.esc_in_us = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      _x = self
      start = end
      end += 22
      (_x.motors_state, _x.control_mode, _x.firmware_time, _x.attitude_mode, _x.lyapunov,) = _get_struct_BiQBd().unpack(str[start:end])
      self.motors_state = bool(self.motors_state)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_28d = None
def _get_struct_28d():
    global _struct_28d
    if _struct_28d is None:
        _struct_28d = struct.Struct("<28d")
    return _struct_28d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_BiQBd = None
def _get_struct_BiQBd():
    global _struct_BiQBd
    if _struct_BiQBd is None:
        _struct_BiQBd = struct.Struct("<BiQBd")
    return _struct_BiQBd
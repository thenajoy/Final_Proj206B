// Auto-generated. Do not edit!

// (in-package qrotor_firmware.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Log {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.euler = null;
      this.body_rates = null;
      this.linear_acceleration = null;
      this.angular_acceleration = null;
      this.cmd_euler = null;
      this.thrust = null;
      this.moment = null;
      this.position = null;
      this.velocity = null;
      this.loop_rate = null;
      this.voltage = null;
      this.current = null;
      this.esc_in_us = null;
      this.motors_state = null;
      this.control_mode = null;
      this.firmware_time = null;
      this.attitude_mode = null;
      this.lyapunov = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('euler')) {
        this.euler = initObj.euler
      }
      else {
        this.euler = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('body_rates')) {
        this.body_rates = initObj.body_rates
      }
      else {
        this.body_rates = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('linear_acceleration')) {
        this.linear_acceleration = initObj.linear_acceleration
      }
      else {
        this.linear_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_acceleration')) {
        this.angular_acceleration = initObj.angular_acceleration
      }
      else {
        this.angular_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('cmd_euler')) {
        this.cmd_euler = initObj.cmd_euler
      }
      else {
        this.cmd_euler = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('moment')) {
        this.moment = initObj.moment
      }
      else {
        this.moment = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('loop_rate')) {
        this.loop_rate = initObj.loop_rate
      }
      else {
        this.loop_rate = 0.0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('esc_in_us')) {
        this.esc_in_us = initObj.esc_in_us
      }
      else {
        this.esc_in_us = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('motors_state')) {
        this.motors_state = initObj.motors_state
      }
      else {
        this.motors_state = false;
      }
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('firmware_time')) {
        this.firmware_time = initObj.firmware_time
      }
      else {
        this.firmware_time = 0;
      }
      if (initObj.hasOwnProperty('attitude_mode')) {
        this.attitude_mode = initObj.attitude_mode
      }
      else {
        this.attitude_mode = 0;
      }
      if (initObj.hasOwnProperty('lyapunov')) {
        this.lyapunov = initObj.lyapunov
      }
      else {
        this.lyapunov = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Log
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [euler]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.euler, buffer, bufferOffset);
    // Serialize message field [body_rates]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.body_rates, buffer, bufferOffset);
    // Serialize message field [linear_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_acceleration, buffer, bufferOffset);
    // Serialize message field [angular_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_acceleration, buffer, bufferOffset);
    // Serialize message field [cmd_euler]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.cmd_euler, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [moment]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.moment, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [loop_rate]
    bufferOffset = _serializer.float64(obj.loop_rate, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float64(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Check that the constant length array field [esc_in_us] has the right length
    if (obj.esc_in_us.length !== 4) {
      throw new Error('Unable to serialize array field esc_in_us - length must be 4')
    }
    // Serialize message field [esc_in_us]
    bufferOffset = _arraySerializer.float64(obj.esc_in_us, buffer, bufferOffset, 4);
    // Serialize message field [motors_state]
    bufferOffset = _serializer.bool(obj.motors_state, buffer, bufferOffset);
    // Serialize message field [control_mode]
    bufferOffset = _serializer.int32(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [firmware_time]
    bufferOffset = _serializer.uint64(obj.firmware_time, buffer, bufferOffset);
    // Serialize message field [attitude_mode]
    bufferOffset = _serializer.uint8(obj.attitude_mode, buffer, bufferOffset);
    // Serialize message field [lyapunov]
    bufferOffset = _serializer.float64(obj.lyapunov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Log
    let len;
    let data = new Log(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [euler]
    data.euler = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_rates]
    data.body_rates = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration]
    data.linear_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_acceleration]
    data.angular_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmd_euler]
    data.cmd_euler = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [moment]
    data.moment = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [loop_rate]
    data.loop_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [esc_in_us]
    data.esc_in_us = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [motors_state]
    data.motors_state = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [firmware_time]
    data.firmware_time = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [attitude_mode]
    data.attitude_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lyapunov]
    data.lyapunov = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 278;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qrotor_firmware/Log';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '479e80fd954016432b0609f599bc1f43';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Quadrotor Onboard Log 
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Log(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.euler !== undefined) {
      resolved.euler = geometry_msgs.msg.Vector3.Resolve(msg.euler)
    }
    else {
      resolved.euler = new geometry_msgs.msg.Vector3()
    }

    if (msg.body_rates !== undefined) {
      resolved.body_rates = geometry_msgs.msg.Vector3.Resolve(msg.body_rates)
    }
    else {
      resolved.body_rates = new geometry_msgs.msg.Vector3()
    }

    if (msg.linear_acceleration !== undefined) {
      resolved.linear_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration)
    }
    else {
      resolved.linear_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_acceleration !== undefined) {
      resolved.angular_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.angular_acceleration)
    }
    else {
      resolved.angular_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.cmd_euler !== undefined) {
      resolved.cmd_euler = geometry_msgs.msg.Vector3.Resolve(msg.cmd_euler)
    }
    else {
      resolved.cmd_euler = new geometry_msgs.msg.Vector3()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.moment !== undefined) {
      resolved.moment = geometry_msgs.msg.Vector3.Resolve(msg.moment)
    }
    else {
      resolved.moment = new geometry_msgs.msg.Vector3()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.loop_rate !== undefined) {
      resolved.loop_rate = msg.loop_rate;
    }
    else {
      resolved.loop_rate = 0.0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.esc_in_us !== undefined) {
      resolved.esc_in_us = msg.esc_in_us;
    }
    else {
      resolved.esc_in_us = new Array(4).fill(0)
    }

    if (msg.motors_state !== undefined) {
      resolved.motors_state = msg.motors_state;
    }
    else {
      resolved.motors_state = false
    }

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.firmware_time !== undefined) {
      resolved.firmware_time = msg.firmware_time;
    }
    else {
      resolved.firmware_time = 0
    }

    if (msg.attitude_mode !== undefined) {
      resolved.attitude_mode = msg.attitude_mode;
    }
    else {
      resolved.attitude_mode = 0
    }

    if (msg.lyapunov !== undefined) {
      resolved.lyapunov = msg.lyapunov;
    }
    else {
      resolved.lyapunov = 0.0
    }

    return resolved;
    }
};

module.exports = Log;

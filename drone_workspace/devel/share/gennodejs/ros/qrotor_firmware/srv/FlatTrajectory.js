// Auto-generated. Do not edit!

// (in-package qrotor_firmware.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class FlatTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.center = null;
      this.radius = null;
      this.phase = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('phase')) {
        this.phase = initObj.phase
      }
      else {
        this.phase = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlatTrajectoryRequest
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.radius, buffer, bufferOffset);
    // Serialize message field [phase]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.phase, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlatTrajectoryRequest
    let len;
    let data = new FlatTrajectoryRequest(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [phase]
    data.phase = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 73;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qrotor_firmware/FlatTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '15191a9c73b67809e5b2cf4690babc5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # set Trajectory type
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
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlatTrajectoryRequest(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Vector3.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Vector3()
    }

    if (msg.radius !== undefined) {
      resolved.radius = geometry_msgs.msg.Vector3.Resolve(msg.radius)
    }
    else {
      resolved.radius = new geometry_msgs.msg.Vector3()
    }

    if (msg.phase !== undefined) {
      resolved.phase = geometry_msgs.msg.Vector3.Resolve(msg.phase)
    }
    else {
      resolved.phase = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

// Constants for message
FlatTrajectoryRequest.Constants = {
  TRAJECTORY_STRAIGHT_LINE: 0,
  TRAJECTORY_CIRCLE_2D: 1,
  TRAJECTORY_CIRCLE_3D: 2,
  TRAJECTORY_ELLIPSE_2D: 3,
  TRAJECTORY_ELLIPSE_3D: 4,
}

class FlatTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlatTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlatTrajectoryResponse
    let len;
    let data = new FlatTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qrotor_firmware/FlatTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlatTrajectoryResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: FlatTrajectoryRequest,
  Response: FlatTrajectoryResponse,
  md5sum() { return 'df0bac3bd179de8433b17a584a39b688'; },
  datatype() { return 'qrotor_firmware/FlatTrajectory'; }
};

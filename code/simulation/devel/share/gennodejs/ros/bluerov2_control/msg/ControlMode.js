// Auto-generated. Do not edit!

// (in-package bluerov2_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ControlMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlMode
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlMode
    let len;
    let data = new ControlMode(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bluerov2_control/ControlMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c56dad23c66f87b189d2dc2a882cdf21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 OFF=0
    uint8 IDLE=1
    uint8 ACCELTELEOP=2
    uint8 VELTELEOP=3
    uint8 HOLDPOSITION=4
    uint8 AUTOPILOT=5
    uint8 LOSGUIDANCE=6
    uint8 ABORT=7
    
    uint8 mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlMode(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

// Constants for message
ControlMode.Constants = {
  OFF: 0,
  IDLE: 1,
  ACCELTELEOP: 2,
  VELTELEOP: 3,
  HOLDPOSITION: 4,
  AUTOPILOT: 5,
  LOSGUIDANCE: 6,
  ABORT: 7,
}

module.exports = ControlMode;

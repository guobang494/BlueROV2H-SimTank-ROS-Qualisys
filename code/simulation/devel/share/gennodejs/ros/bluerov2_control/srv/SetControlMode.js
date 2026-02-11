// Auto-generated. Do not edit!

// (in-package bluerov2_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlMode = require('../msg/ControlMode.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetControlModeRequest {
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
        this.mode = new ControlMode();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControlModeRequest
    // Serialize message field [mode]
    bufferOffset = ControlMode.serialize(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeRequest
    let len;
    let data = new SetControlModeRequest(null);
    // Deserialize message field [mode]
    data.mode = ControlMode.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bluerov2_control/SetControlModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ccbf3c76b075baeeeac2d4c34686ba68';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bluerov2_control/ControlMode mode
    
    ================================================================================
    MSG: bluerov2_control/ControlMode
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
    const resolved = new SetControlModeRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = ControlMode.Resolve(msg.mode)
    }
    else {
      resolved.mode = new ControlMode()
    }

    return resolved;
    }
};

class SetControlModeResponse {
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
    // Serializes a message object of type SetControlModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeResponse
    let len;
    let data = new SetControlModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bluerov2_control/SetControlModeResponse';
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
    const resolved = new SetControlModeResponse(null);
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
  Request: SetControlModeRequest,
  Response: SetControlModeResponse,
  md5sum() { return '2b04d4d97f408d2de1f22ad0896570fc'; },
  datatype() { return 'bluerov2_control/SetControlMode'; }
};

// Auto-generated. Do not edit!

// (in-package bluerov2_dobmpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.current_pose = null;
      this.current_twist = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('current_pose')) {
        this.current_pose = initObj.current_pose
      }
      else {
        this.current_pose = [];
      }
      if (initObj.hasOwnProperty('current_twist')) {
        this.current_twist = initObj.current_twist
      }
      else {
        this.current_twist = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [current_pose]
    // Serialize the length for message field [current_pose]
    bufferOffset = _serializer.uint32(obj.current_pose.length, buffer, bufferOffset);
    obj.current_pose.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [current_twist]
    // Serialize the length for message field [current_twist]
    bufferOffset = _serializer.uint32(obj.current_twist.length, buffer, bufferOffset);
    obj.current_twist.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Twist.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pose
    let len;
    let data = new Pose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [current_pose]
    // Deserialize array length for message field [current_pose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.current_pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.current_pose[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [current_twist]
    // Deserialize array length for message field [current_twist]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.current_twist = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.current_twist[i] = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 56 * object.current_pose.length;
    length += 48 * object.current_twist.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bluerov2_dobmpc/Pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2658de99fec3ecc40cbc0db789c4bb3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Pose[] current_pose
    geometry_msgs/Twist[] current_twist
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3 linear
    Vector3 angular
    
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
    const resolved = new Pose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.current_pose !== undefined) {
      resolved.current_pose = new Array(msg.current_pose.length);
      for (let i = 0; i < resolved.current_pose.length; ++i) {
        resolved.current_pose[i] = geometry_msgs.msg.Pose.Resolve(msg.current_pose[i]);
      }
    }
    else {
      resolved.current_pose = []
    }

    if (msg.current_twist !== undefined) {
      resolved.current_twist = new Array(msg.current_twist.length);
      for (let i = 0; i < resolved.current_twist.length; ++i) {
        resolved.current_twist[i] = geometry_msgs.msg.Twist.Resolve(msg.current_twist[i]);
      }
    }
    else {
      resolved.current_twist = []
    }

    return resolved;
    }
};

module.exports = Pose;

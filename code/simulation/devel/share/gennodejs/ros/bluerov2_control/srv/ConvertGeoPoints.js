// Auto-generated. Do not edit!

// (in-package bluerov2_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geographic_msgs = _finder('geographic_msgs');

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ConvertGeoPointsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.geopoints = null;
    }
    else {
      if (initObj.hasOwnProperty('geopoints')) {
        this.geopoints = initObj.geopoints
      }
      else {
        this.geopoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConvertGeoPointsRequest
    // Serialize message field [geopoints]
    // Serialize the length for message field [geopoints]
    bufferOffset = _serializer.uint32(obj.geopoints.length, buffer, bufferOffset);
    obj.geopoints.forEach((val) => {
      bufferOffset = geographic_msgs.msg.GeoPoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConvertGeoPointsRequest
    let len;
    let data = new ConvertGeoPointsRequest(null);
    // Deserialize message field [geopoints]
    // Deserialize array length for message field [geopoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.geopoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.geopoints[i] = geographic_msgs.msg.GeoPoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.geopoints.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bluerov2_control/ConvertGeoPointsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8bbeac8c86c7cfa3d22cd50dc8855fa5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geographic_msgs/GeoPoint[] geopoints
    
    ================================================================================
    MSG: geographic_msgs/GeoPoint
    # Geographic point, using the WGS 84 reference ellipsoid.
    
    # Latitude [degrees]. Positive is north of equator; negative is south
    # (-90 <= latitude <= +90).
    float64 latitude
    
    # Longitude [degrees]. Positive is east of prime meridian; negative is
    # west (-180 <= longitude <= +180). At the poles, latitude is -90 or
    # +90, and longitude is irrelevant, but must be in range.
    float64 longitude
    
    # Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
    float64 altitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConvertGeoPointsRequest(null);
    if (msg.geopoints !== undefined) {
      resolved.geopoints = new Array(msg.geopoints.length);
      for (let i = 0; i < resolved.geopoints.length; ++i) {
        resolved.geopoints[i] = geographic_msgs.msg.GeoPoint.Resolve(msg.geopoints[i]);
      }
    }
    else {
      resolved.geopoints = []
    }

    return resolved;
    }
};

class ConvertGeoPointsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.utmpoints = null;
    }
    else {
      if (initObj.hasOwnProperty('utmpoints')) {
        this.utmpoints = initObj.utmpoints
      }
      else {
        this.utmpoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConvertGeoPointsResponse
    // Serialize message field [utmpoints]
    // Serialize the length for message field [utmpoints]
    bufferOffset = _serializer.uint32(obj.utmpoints.length, buffer, bufferOffset);
    obj.utmpoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConvertGeoPointsResponse
    let len;
    let data = new ConvertGeoPointsResponse(null);
    // Deserialize message field [utmpoints]
    // Deserialize array length for message field [utmpoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.utmpoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.utmpoints[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.utmpoints.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bluerov2_control/ConvertGeoPointsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99c794da9bcae9f599e7efd1c9cf51d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] utmpoints
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new ConvertGeoPointsResponse(null);
    if (msg.utmpoints !== undefined) {
      resolved.utmpoints = new Array(msg.utmpoints.length);
      for (let i = 0; i < resolved.utmpoints.length; ++i) {
        resolved.utmpoints[i] = geometry_msgs.msg.Point.Resolve(msg.utmpoints[i]);
      }
    }
    else {
      resolved.utmpoints = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ConvertGeoPointsRequest,
  Response: ConvertGeoPointsResponse,
  md5sum() { return 'a34bd81e20a7f59208cae9a3910e02a9'; },
  datatype() { return 'bluerov2_control/ConvertGeoPoints'; }
};

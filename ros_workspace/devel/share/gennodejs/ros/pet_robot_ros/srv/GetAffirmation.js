// Auto-generated. Do not edit!

// (in-package pet_robot_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetAffirmationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAffirmationRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAffirmationRequest
    let len;
    let data = new GetAffirmationRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/GetAffirmationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request a positive affirmation
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAffirmationRequest(null);
    return resolved;
    }
};

class GetAffirmationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.affirmation = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('affirmation')) {
        this.affirmation = initObj.affirmation
      }
      else {
        this.affirmation = '';
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAffirmationResponse
    // Serialize message field [affirmation]
    bufferOffset = _serializer.string(obj.affirmation, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAffirmationResponse
    let len;
    let data = new GetAffirmationResponse(null);
    // Deserialize message field [affirmation]
    data.affirmation = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.affirmation.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/GetAffirmationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '176b3ebe98603e26ffc647c2f65b7d30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    string affirmation
    time timestamp
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAffirmationResponse(null);
    if (msg.affirmation !== undefined) {
      resolved.affirmation = msg.affirmation;
    }
    else {
      resolved.affirmation = ''
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAffirmationRequest,
  Response: GetAffirmationResponse,
  md5sum() { return '176b3ebe98603e26ffc647c2f65b7d30'; },
  datatype() { return 'pet_robot_ros/GetAffirmation'; }
};

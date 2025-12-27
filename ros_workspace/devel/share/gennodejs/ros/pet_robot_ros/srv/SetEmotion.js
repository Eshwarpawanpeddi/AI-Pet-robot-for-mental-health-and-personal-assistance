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

class SetEmotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emotion = null;
      this.intensity = null;
    }
    else {
      if (initObj.hasOwnProperty('emotion')) {
        this.emotion = initObj.emotion
      }
      else {
        this.emotion = '';
      }
      if (initObj.hasOwnProperty('intensity')) {
        this.intensity = initObj.intensity
      }
      else {
        this.intensity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEmotionRequest
    // Serialize message field [emotion]
    bufferOffset = _serializer.string(obj.emotion, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.float32(obj.intensity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEmotionRequest
    let len;
    let data = new SetEmotionRequest(null);
    // Deserialize message field [emotion]
    data.emotion = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.emotion.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/SetEmotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55951f00a9a7dc87dad4cca95af9a536';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request to set robot's emotion
    string emotion  # happy, sad, anxious, stressed, tired, angry, neutral
    float32 intensity  # 0.0 to 1.0
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetEmotionRequest(null);
    if (msg.emotion !== undefined) {
      resolved.emotion = msg.emotion;
    }
    else {
      resolved.emotion = ''
    }

    if (msg.intensity !== undefined) {
      resolved.intensity = msg.intensity;
    }
    else {
      resolved.intensity = 0.0
    }

    return resolved;
    }
};

class SetEmotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEmotionResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEmotionResponse
    let len;
    let data = new SetEmotionResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/SetEmotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetEmotionResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetEmotionRequest,
  Response: SetEmotionResponse,
  md5sum() { return 'b4511e9a0d7a1edc71467ea13a9b50a5'; },
  datatype() { return 'pet_robot_ros/SetEmotion'; }
};

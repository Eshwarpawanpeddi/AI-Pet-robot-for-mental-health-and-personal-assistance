// Auto-generated. Do not edit!

// (in-package pet_robot_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Emotion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emotion = null;
      this.intensity = null;
      this.timestamp = null;
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
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Emotion
    // Serialize message field [emotion]
    bufferOffset = _serializer.string(obj.emotion, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.float32(obj.intensity, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Emotion
    let len;
    let data = new Emotion(null);
    // Deserialize message field [emotion]
    data.emotion = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.emotion.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pet_robot_ros/Emotion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e7af84fb54cd74beec900c75b72aa6ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Emotion message for robot's current emotional state
    string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis
    float32 intensity  # 0.0 to 1.0
    time timestamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Emotion(null);
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

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = Emotion;

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

class MoodLog {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mood = null;
      this.intensity = null;
      this.notes = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('mood')) {
        this.mood = initObj.mood
      }
      else {
        this.mood = '';
      }
      if (initObj.hasOwnProperty('intensity')) {
        this.intensity = initObj.intensity
      }
      else {
        this.intensity = 0;
      }
      if (initObj.hasOwnProperty('notes')) {
        this.notes = initObj.notes
      }
      else {
        this.notes = '';
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
    // Serializes a message object of type MoodLog
    // Serialize message field [mood]
    bufferOffset = _serializer.string(obj.mood, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.int32(obj.intensity, buffer, bufferOffset);
    // Serialize message field [notes]
    bufferOffset = _serializer.string(obj.notes, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoodLog
    let len;
    let data = new MoodLog(null);
    // Deserialize message field [mood]
    data.mood = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [notes]
    data.notes = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mood.length;
    length += object.notes.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pet_robot_ros/MoodLog';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4584f6b7d371bdba788c5cd148510811';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Mood log entry for mental health tracking
    string mood  # anxious, sad, happy, stressed, tired, angry, neutral
    int32 intensity  # 1-10 scale
    string notes
    time timestamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoodLog(null);
    if (msg.mood !== undefined) {
      resolved.mood = msg.mood;
    }
    else {
      resolved.mood = ''
    }

    if (msg.intensity !== undefined) {
      resolved.intensity = msg.intensity;
    }
    else {
      resolved.intensity = 0
    }

    if (msg.notes !== undefined) {
      resolved.notes = msg.notes;
    }
    else {
      resolved.notes = ''
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

module.exports = MoodLog;

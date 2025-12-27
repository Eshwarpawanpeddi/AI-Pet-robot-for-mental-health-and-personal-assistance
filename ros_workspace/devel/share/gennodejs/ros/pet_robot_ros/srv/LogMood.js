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

class LogMoodRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mood = null;
      this.intensity = null;
      this.notes = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LogMoodRequest
    // Serialize message field [mood]
    bufferOffset = _serializer.string(obj.mood, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.int32(obj.intensity, buffer, bufferOffset);
    // Serialize message field [notes]
    bufferOffset = _serializer.string(obj.notes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LogMoodRequest
    let len;
    let data = new LogMoodRequest(null);
    // Deserialize message field [mood]
    data.mood = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [notes]
    data.notes = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mood.length;
    length += object.notes.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/LogMoodRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a74e8fb381bedf7ffd594eae7cf7b5c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request to log mood
    string mood  # anxious, sad, happy, stressed, tired, angry, neutral
    int32 intensity  # 1-10 scale
    string notes
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LogMoodRequest(null);
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

    return resolved;
    }
};

class LogMoodResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.response = null;
      this.suggestions = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
      if (initObj.hasOwnProperty('suggestions')) {
        this.suggestions = initObj.suggestions
      }
      else {
        this.suggestions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LogMoodResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    // Serialize message field [suggestions]
    bufferOffset = _arraySerializer.string(obj.suggestions, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LogMoodResponse
    let len;
    let data = new LogMoodResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [suggestions]
    data.suggestions = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.response.length;
    object.suggestions.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pet_robot_ros/LogMoodResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2981315bcdd00a3184bc6d2282ec1f55';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    bool success
    string response  # Supportive response from robot
    string[] suggestions  # Suggested coping activities
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LogMoodResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    if (msg.suggestions !== undefined) {
      resolved.suggestions = msg.suggestions;
    }
    else {
      resolved.suggestions = []
    }

    return resolved;
    }
};

module.exports = {
  Request: LogMoodRequest,
  Response: LogMoodResponse,
  md5sum() { return 'b43c763a2a7bf67b61c6235ef921779c'; },
  datatype() { return 'pet_robot_ros/LogMood'; }
};

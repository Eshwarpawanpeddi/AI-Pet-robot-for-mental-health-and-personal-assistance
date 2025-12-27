// Auto-generated. Do not edit!

// (in-package pet_robot_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Emotion = require('./Emotion.js');
let SensorData = require('./SensorData.js');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emotion = null;
      this.is_speaking = null;
      this.is_listening = null;
      this.battery_level = null;
      this.sensor_data = null;
    }
    else {
      if (initObj.hasOwnProperty('emotion')) {
        this.emotion = initObj.emotion
      }
      else {
        this.emotion = new Emotion();
      }
      if (initObj.hasOwnProperty('is_speaking')) {
        this.is_speaking = initObj.is_speaking
      }
      else {
        this.is_speaking = false;
      }
      if (initObj.hasOwnProperty('is_listening')) {
        this.is_listening = initObj.is_listening
      }
      else {
        this.is_listening = false;
      }
      if (initObj.hasOwnProperty('battery_level')) {
        this.battery_level = initObj.battery_level
      }
      else {
        this.battery_level = 0;
      }
      if (initObj.hasOwnProperty('sensor_data')) {
        this.sensor_data = initObj.sensor_data
      }
      else {
        this.sensor_data = new SensorData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [emotion]
    bufferOffset = Emotion.serialize(obj.emotion, buffer, bufferOffset);
    // Serialize message field [is_speaking]
    bufferOffset = _serializer.bool(obj.is_speaking, buffer, bufferOffset);
    // Serialize message field [is_listening]
    bufferOffset = _serializer.bool(obj.is_listening, buffer, bufferOffset);
    // Serialize message field [battery_level]
    bufferOffset = _serializer.int32(obj.battery_level, buffer, bufferOffset);
    // Serialize message field [sensor_data]
    bufferOffset = SensorData.serialize(obj.sensor_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [emotion]
    data.emotion = Emotion.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_speaking]
    data.is_speaking = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_listening]
    data.is_listening = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [battery_level]
    data.battery_level = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sensor_data]
    data.sensor_data = SensorData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Emotion.getMessageSize(object.emotion);
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pet_robot_ros/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23f43ffc60bb2cce618de7427a61ab01';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Complete robot state
    Emotion emotion
    bool is_speaking
    bool is_listening
    int32 battery_level  # 0-100
    SensorData sensor_data
    
    ================================================================================
    MSG: pet_robot_ros/Emotion
    # Emotion message for robot's current emotional state
    string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis
    float32 intensity  # 0.0 to 1.0
    time timestamp
    
    ================================================================================
    MSG: pet_robot_ros/SensorData
    # Sensor data from robot
    bool touch_detected
    float32 distance  # cm
    float32 temperature  # Celsius
    time timestamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotState(null);
    if (msg.emotion !== undefined) {
      resolved.emotion = Emotion.Resolve(msg.emotion)
    }
    else {
      resolved.emotion = new Emotion()
    }

    if (msg.is_speaking !== undefined) {
      resolved.is_speaking = msg.is_speaking;
    }
    else {
      resolved.is_speaking = false
    }

    if (msg.is_listening !== undefined) {
      resolved.is_listening = msg.is_listening;
    }
    else {
      resolved.is_listening = false
    }

    if (msg.battery_level !== undefined) {
      resolved.battery_level = msg.battery_level;
    }
    else {
      resolved.battery_level = 0
    }

    if (msg.sensor_data !== undefined) {
      resolved.sensor_data = SensorData.Resolve(msg.sensor_data)
    }
    else {
      resolved.sensor_data = new SensorData()
    }

    return resolved;
    }
};

module.exports = RobotState;

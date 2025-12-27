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

class SensorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.touch_detected = null;
      this.distance = null;
      this.temperature = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('touch_detected')) {
        this.touch_detected = initObj.touch_detected
      }
      else {
        this.touch_detected = false;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
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
    // Serializes a message object of type SensorData
    // Serialize message field [touch_detected]
    bufferOffset = _serializer.bool(obj.touch_detected, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorData
    let len;
    let data = new SensorData(null);
    // Deserialize message field [touch_detected]
    data.touch_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pet_robot_ros/SensorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a89da8b484867ca2bc71805d9719a4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SensorData(null);
    if (msg.touch_detected !== undefined) {
      resolved.touch_detected = msg.touch_detected;
    }
    else {
      resolved.touch_detected = false
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
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

module.exports = SensorData;

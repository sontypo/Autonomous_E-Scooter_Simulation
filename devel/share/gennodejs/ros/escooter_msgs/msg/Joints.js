// Auto-generated. Do not edit!

// (in-package escooter_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Joints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle_bar = null;
      this.rotational_plate = null;
      this.front_wheel = null;
    }
    else {
      if (initObj.hasOwnProperty('handle_bar')) {
        this.handle_bar = initObj.handle_bar
      }
      else {
        this.handle_bar = 0.0;
      }
      if (initObj.hasOwnProperty('rotational_plate')) {
        this.rotational_plate = initObj.rotational_plate
      }
      else {
        this.rotational_plate = 0.0;
      }
      if (initObj.hasOwnProperty('front_wheel')) {
        this.front_wheel = initObj.front_wheel
      }
      else {
        this.front_wheel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Joints
    // Serialize message field [handle_bar]
    bufferOffset = _serializer.float64(obj.handle_bar, buffer, bufferOffset);
    // Serialize message field [rotational_plate]
    bufferOffset = _serializer.float64(obj.rotational_plate, buffer, bufferOffset);
    // Serialize message field [front_wheel]
    bufferOffset = _serializer.float64(obj.front_wheel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Joints
    let len;
    let data = new Joints(null);
    // Deserialize message field [handle_bar]
    data.handle_bar = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotational_plate]
    data.rotational_plate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_wheel]
    data.front_wheel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'escooter_msgs/Joints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3e5fbcf0da70ecde669dea5a74f6f7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 handle_bar
    float64 rotational_plate
    float64 front_wheel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Joints(null);
    if (msg.handle_bar !== undefined) {
      resolved.handle_bar = msg.handle_bar;
    }
    else {
      resolved.handle_bar = 0.0
    }

    if (msg.rotational_plate !== undefined) {
      resolved.rotational_plate = msg.rotational_plate;
    }
    else {
      resolved.rotational_plate = 0.0
    }

    if (msg.front_wheel !== undefined) {
      resolved.front_wheel = msg.front_wheel;
    }
    else {
      resolved.front_wheel = 0.0
    }

    return resolved;
    }
};

module.exports = Joints;

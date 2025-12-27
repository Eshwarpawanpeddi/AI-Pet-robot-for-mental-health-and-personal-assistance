; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-msg)


;//! \htmlinclude RobotState.msg.html

(cl:defclass <RobotState> (roslisp-msg-protocol:ros-message)
  ((emotion
    :reader emotion
    :initarg :emotion
    :type pet_robot_ros-msg:Emotion
    :initform (cl:make-instance 'pet_robot_ros-msg:Emotion))
   (is_speaking
    :reader is_speaking
    :initarg :is_speaking
    :type cl:boolean
    :initform cl:nil)
   (is_listening
    :reader is_listening
    :initarg :is_listening
    :type cl:boolean
    :initform cl:nil)
   (battery_level
    :reader battery_level
    :initarg :battery_level
    :type cl:integer
    :initform 0)
   (sensor_data
    :reader sensor_data
    :initarg :sensor_data
    :type pet_robot_ros-msg:SensorData
    :initform (cl:make-instance 'pet_robot_ros-msg:SensorData)))
)

(cl:defclass RobotState (<RobotState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-msg:<RobotState> is deprecated: use pet_robot_ros-msg:RobotState instead.")))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:emotion-val is deprecated.  Use pet_robot_ros-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'is_speaking-val :lambda-list '(m))
(cl:defmethod is_speaking-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:is_speaking-val is deprecated.  Use pet_robot_ros-msg:is_speaking instead.")
  (is_speaking m))

(cl:ensure-generic-function 'is_listening-val :lambda-list '(m))
(cl:defmethod is_listening-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:is_listening-val is deprecated.  Use pet_robot_ros-msg:is_listening instead.")
  (is_listening m))

(cl:ensure-generic-function 'battery_level-val :lambda-list '(m))
(cl:defmethod battery_level-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:battery_level-val is deprecated.  Use pet_robot_ros-msg:battery_level instead.")
  (battery_level m))

(cl:ensure-generic-function 'sensor_data-val :lambda-list '(m))
(cl:defmethod sensor_data-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:sensor_data-val is deprecated.  Use pet_robot_ros-msg:sensor_data instead.")
  (sensor_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotState>) ostream)
  "Serializes a message object of type '<RobotState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emotion) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_speaking) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_listening) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'battery_level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensor_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotState>) istream)
  "Deserializes a message object of type '<RobotState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emotion) istream)
    (cl:setf (cl:slot-value msg 'is_speaking) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_listening) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_level) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensor_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotState>)))
  "Returns string type for a message object of type '<RobotState>"
  "pet_robot_ros/RobotState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotState)))
  "Returns string type for a message object of type 'RobotState"
  "pet_robot_ros/RobotState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotState>)))
  "Returns md5sum for a message object of type '<RobotState>"
  "23f43ffc60bb2cce618de7427a61ab01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotState)))
  "Returns md5sum for a message object of type 'RobotState"
  "23f43ffc60bb2cce618de7427a61ab01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotState>)))
  "Returns full string definition for message of type '<RobotState>"
  (cl:format cl:nil "# Complete robot state~%Emotion emotion~%bool is_speaking~%bool is_listening~%int32 battery_level  # 0-100~%SensorData sensor_data~%~%================================================================================~%MSG: pet_robot_ros/Emotion~%# Emotion message for robot's current emotional state~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis~%float32 intensity  # 0.0 to 1.0~%time timestamp~%~%================================================================================~%MSG: pet_robot_ros/SensorData~%# Sensor data from robot~%bool touch_detected~%float32 distance  # cm~%float32 temperature  # Celsius~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotState)))
  "Returns full string definition for message of type 'RobotState"
  (cl:format cl:nil "# Complete robot state~%Emotion emotion~%bool is_speaking~%bool is_listening~%int32 battery_level  # 0-100~%SensorData sensor_data~%~%================================================================================~%MSG: pet_robot_ros/Emotion~%# Emotion message for robot's current emotional state~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis~%float32 intensity  # 0.0 to 1.0~%time timestamp~%~%================================================================================~%MSG: pet_robot_ros/SensorData~%# Sensor data from robot~%bool touch_detected~%float32 distance  # cm~%float32 temperature  # Celsius~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emotion))
     1
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensor_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotState>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotState
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':is_speaking (is_speaking msg))
    (cl:cons ':is_listening (is_listening msg))
    (cl:cons ':battery_level (battery_level msg))
    (cl:cons ':sensor_data (sensor_data msg))
))

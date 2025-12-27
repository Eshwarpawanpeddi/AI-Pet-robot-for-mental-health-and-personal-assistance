; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-msg)


;//! \htmlinclude SensorData.msg.html

(cl:defclass <SensorData> (roslisp-msg-protocol:ros-message)
  ((touch_detected
    :reader touch_detected
    :initarg :touch_detected
    :type cl:boolean
    :initform cl:nil)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0))
)

(cl:defclass SensorData (<SensorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-msg:<SensorData> is deprecated: use pet_robot_ros-msg:SensorData instead.")))

(cl:ensure-generic-function 'touch_detected-val :lambda-list '(m))
(cl:defmethod touch_detected-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:touch_detected-val is deprecated.  Use pet_robot_ros-msg:touch_detected instead.")
  (touch_detected m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:distance-val is deprecated.  Use pet_robot_ros-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:temperature-val is deprecated.  Use pet_robot_ros-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:timestamp-val is deprecated.  Use pet_robot_ros-msg:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorData>) ostream)
  "Serializes a message object of type '<SensorData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'touch_detected) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorData>) istream)
  "Deserializes a message object of type '<SensorData>"
    (cl:setf (cl:slot-value msg 'touch_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorData>)))
  "Returns string type for a message object of type '<SensorData>"
  "pet_robot_ros/SensorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorData)))
  "Returns string type for a message object of type 'SensorData"
  "pet_robot_ros/SensorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorData>)))
  "Returns md5sum for a message object of type '<SensorData>"
  "5a89da8b484867ca2bc71805d9719a4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorData)))
  "Returns md5sum for a message object of type 'SensorData"
  "5a89da8b484867ca2bc71805d9719a4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorData>)))
  "Returns full string definition for message of type '<SensorData>"
  (cl:format cl:nil "# Sensor data from robot~%bool touch_detected~%float32 distance  # cm~%float32 temperature  # Celsius~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorData)))
  "Returns full string definition for message of type 'SensorData"
  (cl:format cl:nil "# Sensor data from robot~%bool touch_detected~%float32 distance  # cm~%float32 temperature  # Celsius~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorData>))
  (cl:+ 0
     1
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorData>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorData
    (cl:cons ':touch_detected (touch_detected msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':timestamp (timestamp msg))
))

; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-srv)


;//! \htmlinclude SetEmotion-request.msg.html

(cl:defclass <SetEmotion-request> (roslisp-msg-protocol:ros-message)
  ((emotion
    :reader emotion
    :initarg :emotion
    :type cl:string
    :initform "")
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetEmotion-request (<SetEmotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEmotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEmotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<SetEmotion-request> is deprecated: use pet_robot_ros-srv:SetEmotion-request instead.")))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <SetEmotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:emotion-val is deprecated.  Use pet_robot_ros-srv:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <SetEmotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:intensity-val is deprecated.  Use pet_robot_ros-srv:intensity instead.")
  (intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEmotion-request>) ostream)
  "Serializes a message object of type '<SetEmotion-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'emotion))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'emotion))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'intensity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEmotion-request>) istream)
  "Deserializes a message object of type '<SetEmotion-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'emotion) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'emotion) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'intensity) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEmotion-request>)))
  "Returns string type for a service object of type '<SetEmotion-request>"
  "pet_robot_ros/SetEmotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEmotion-request)))
  "Returns string type for a service object of type 'SetEmotion-request"
  "pet_robot_ros/SetEmotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEmotion-request>)))
  "Returns md5sum for a message object of type '<SetEmotion-request>"
  "b4511e9a0d7a1edc71467ea13a9b50a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEmotion-request)))
  "Returns md5sum for a message object of type 'SetEmotion-request"
  "b4511e9a0d7a1edc71467ea13a9b50a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEmotion-request>)))
  "Returns full string definition for message of type '<SetEmotion-request>"
  (cl:format cl:nil "# Request to set robot's emotion~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral~%float32 intensity  # 0.0 to 1.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEmotion-request)))
  "Returns full string definition for message of type 'SetEmotion-request"
  (cl:format cl:nil "# Request to set robot's emotion~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral~%float32 intensity  # 0.0 to 1.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEmotion-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'emotion))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEmotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEmotion-request
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':intensity (intensity msg))
))
;//! \htmlinclude SetEmotion-response.msg.html

(cl:defclass <SetEmotion-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetEmotion-response (<SetEmotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEmotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEmotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<SetEmotion-response> is deprecated: use pet_robot_ros-srv:SetEmotion-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetEmotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:success-val is deprecated.  Use pet_robot_ros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetEmotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:message-val is deprecated.  Use pet_robot_ros-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEmotion-response>) ostream)
  "Serializes a message object of type '<SetEmotion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEmotion-response>) istream)
  "Deserializes a message object of type '<SetEmotion-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEmotion-response>)))
  "Returns string type for a service object of type '<SetEmotion-response>"
  "pet_robot_ros/SetEmotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEmotion-response)))
  "Returns string type for a service object of type 'SetEmotion-response"
  "pet_robot_ros/SetEmotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEmotion-response>)))
  "Returns md5sum for a message object of type '<SetEmotion-response>"
  "b4511e9a0d7a1edc71467ea13a9b50a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEmotion-response)))
  "Returns md5sum for a message object of type 'SetEmotion-response"
  "b4511e9a0d7a1edc71467ea13a9b50a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEmotion-response>)))
  "Returns full string definition for message of type '<SetEmotion-response>"
  (cl:format cl:nil "# Response~%bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEmotion-response)))
  "Returns full string definition for message of type 'SetEmotion-response"
  (cl:format cl:nil "# Response~%bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEmotion-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEmotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEmotion-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetEmotion)))
  'SetEmotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetEmotion)))
  'SetEmotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEmotion)))
  "Returns string type for a service object of type '<SetEmotion>"
  "pet_robot_ros/SetEmotion")
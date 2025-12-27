; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-msg)


;//! \htmlinclude Emotion.msg.html

(cl:defclass <Emotion> (roslisp-msg-protocol:ros-message)
  ((emotion
    :reader emotion
    :initarg :emotion
    :type cl:string
    :initform "")
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:float
    :initform 0.0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0))
)

(cl:defclass Emotion (<Emotion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Emotion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Emotion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-msg:<Emotion> is deprecated: use pet_robot_ros-msg:Emotion instead.")))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <Emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:emotion-val is deprecated.  Use pet_robot_ros-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <Emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:intensity-val is deprecated.  Use pet_robot_ros-msg:intensity instead.")
  (intensity m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:timestamp-val is deprecated.  Use pet_robot_ros-msg:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Emotion>) ostream)
  "Serializes a message object of type '<Emotion>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Emotion>) istream)
  "Deserializes a message object of type '<Emotion>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Emotion>)))
  "Returns string type for a message object of type '<Emotion>"
  "pet_robot_ros/Emotion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emotion)))
  "Returns string type for a message object of type 'Emotion"
  "pet_robot_ros/Emotion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Emotion>)))
  "Returns md5sum for a message object of type '<Emotion>"
  "e7af84fb54cd74beec900c75b72aa6ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Emotion)))
  "Returns md5sum for a message object of type 'Emotion"
  "e7af84fb54cd74beec900c75b72aa6ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Emotion>)))
  "Returns full string definition for message of type '<Emotion>"
  (cl:format cl:nil "# Emotion message for robot's current emotional state~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis~%float32 intensity  # 0.0 to 1.0~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Emotion)))
  "Returns full string definition for message of type 'Emotion"
  (cl:format cl:nil "# Emotion message for robot's current emotional state~%string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis~%float32 intensity  # 0.0 to 1.0~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Emotion>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'emotion))
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Emotion>))
  "Converts a ROS message object to a list"
  (cl:list 'Emotion
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':intensity (intensity msg))
    (cl:cons ':timestamp (timestamp msg))
))

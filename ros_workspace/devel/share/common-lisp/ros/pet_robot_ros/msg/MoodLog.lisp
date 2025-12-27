; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-msg)


;//! \htmlinclude MoodLog.msg.html

(cl:defclass <MoodLog> (roslisp-msg-protocol:ros-message)
  ((mood
    :reader mood
    :initarg :mood
    :type cl:string
    :initform "")
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:integer
    :initform 0)
   (notes
    :reader notes
    :initarg :notes
    :type cl:string
    :initform "")
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0))
)

(cl:defclass MoodLog (<MoodLog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoodLog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoodLog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-msg:<MoodLog> is deprecated: use pet_robot_ros-msg:MoodLog instead.")))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <MoodLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:mood-val is deprecated.  Use pet_robot_ros-msg:mood instead.")
  (mood m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <MoodLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:intensity-val is deprecated.  Use pet_robot_ros-msg:intensity instead.")
  (intensity m))

(cl:ensure-generic-function 'notes-val :lambda-list '(m))
(cl:defmethod notes-val ((m <MoodLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:notes-val is deprecated.  Use pet_robot_ros-msg:notes instead.")
  (notes m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <MoodLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:timestamp-val is deprecated.  Use pet_robot_ros-msg:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoodLog>) ostream)
  "Serializes a message object of type '<MoodLog>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mood))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mood))
  (cl:let* ((signed (cl:slot-value msg 'intensity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'notes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'notes))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoodLog>) istream)
  "Deserializes a message object of type '<MoodLog>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mood) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mood) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'intensity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'notes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'notes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoodLog>)))
  "Returns string type for a message object of type '<MoodLog>"
  "pet_robot_ros/MoodLog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoodLog)))
  "Returns string type for a message object of type 'MoodLog"
  "pet_robot_ros/MoodLog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoodLog>)))
  "Returns md5sum for a message object of type '<MoodLog>"
  "4584f6b7d371bdba788c5cd148510811")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoodLog)))
  "Returns md5sum for a message object of type 'MoodLog"
  "4584f6b7d371bdba788c5cd148510811")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoodLog>)))
  "Returns full string definition for message of type '<MoodLog>"
  (cl:format cl:nil "# Mood log entry for mental health tracking~%string mood  # anxious, sad, happy, stressed, tired, angry, neutral~%int32 intensity  # 1-10 scale~%string notes~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoodLog)))
  "Returns full string definition for message of type 'MoodLog"
  (cl:format cl:nil "# Mood log entry for mental health tracking~%string mood  # anxious, sad, happy, stressed, tired, angry, neutral~%int32 intensity  # 1-10 scale~%string notes~%time timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoodLog>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mood))
     4
     4 (cl:length (cl:slot-value msg 'notes))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoodLog>))
  "Converts a ROS message object to a list"
  (cl:list 'MoodLog
    (cl:cons ':mood (mood msg))
    (cl:cons ':intensity (intensity msg))
    (cl:cons ':notes (notes msg))
    (cl:cons ':timestamp (timestamp msg))
))

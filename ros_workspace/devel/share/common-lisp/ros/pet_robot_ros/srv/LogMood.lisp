; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-srv)


;//! \htmlinclude LogMood-request.msg.html

(cl:defclass <LogMood-request> (roslisp-msg-protocol:ros-message)
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
    :initform ""))
)

(cl:defclass LogMood-request (<LogMood-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LogMood-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LogMood-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<LogMood-request> is deprecated: use pet_robot_ros-srv:LogMood-request instead.")))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <LogMood-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:mood-val is deprecated.  Use pet_robot_ros-srv:mood instead.")
  (mood m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <LogMood-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:intensity-val is deprecated.  Use pet_robot_ros-srv:intensity instead.")
  (intensity m))

(cl:ensure-generic-function 'notes-val :lambda-list '(m))
(cl:defmethod notes-val ((m <LogMood-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:notes-val is deprecated.  Use pet_robot_ros-srv:notes instead.")
  (notes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LogMood-request>) ostream)
  "Serializes a message object of type '<LogMood-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LogMood-request>) istream)
  "Deserializes a message object of type '<LogMood-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LogMood-request>)))
  "Returns string type for a service object of type '<LogMood-request>"
  "pet_robot_ros/LogMoodRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogMood-request)))
  "Returns string type for a service object of type 'LogMood-request"
  "pet_robot_ros/LogMoodRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LogMood-request>)))
  "Returns md5sum for a message object of type '<LogMood-request>"
  "b43c763a2a7bf67b61c6235ef921779c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LogMood-request)))
  "Returns md5sum for a message object of type 'LogMood-request"
  "b43c763a2a7bf67b61c6235ef921779c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LogMood-request>)))
  "Returns full string definition for message of type '<LogMood-request>"
  (cl:format cl:nil "# Request to log mood~%string mood  # anxious, sad, happy, stressed, tired, angry, neutral~%int32 intensity  # 1-10 scale~%string notes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LogMood-request)))
  "Returns full string definition for message of type 'LogMood-request"
  (cl:format cl:nil "# Request to log mood~%string mood  # anxious, sad, happy, stressed, tired, angry, neutral~%int32 intensity  # 1-10 scale~%string notes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LogMood-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mood))
     4
     4 (cl:length (cl:slot-value msg 'notes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LogMood-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LogMood-request
    (cl:cons ':mood (mood msg))
    (cl:cons ':intensity (intensity msg))
    (cl:cons ':notes (notes msg))
))
;//! \htmlinclude LogMood-response.msg.html

(cl:defclass <LogMood-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (response
    :reader response
    :initarg :response
    :type cl:string
    :initform "")
   (suggestions
    :reader suggestions
    :initarg :suggestions
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass LogMood-response (<LogMood-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LogMood-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LogMood-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<LogMood-response> is deprecated: use pet_robot_ros-srv:LogMood-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LogMood-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:success-val is deprecated.  Use pet_robot_ros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <LogMood-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:response-val is deprecated.  Use pet_robot_ros-srv:response instead.")
  (response m))

(cl:ensure-generic-function 'suggestions-val :lambda-list '(m))
(cl:defmethod suggestions-val ((m <LogMood-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:suggestions-val is deprecated.  Use pet_robot_ros-srv:suggestions instead.")
  (suggestions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LogMood-response>) ostream)
  "Serializes a message object of type '<LogMood-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'suggestions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'suggestions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LogMood-response>) istream)
  "Deserializes a message object of type '<LogMood-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'suggestions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'suggestions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LogMood-response>)))
  "Returns string type for a service object of type '<LogMood-response>"
  "pet_robot_ros/LogMoodResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogMood-response)))
  "Returns string type for a service object of type 'LogMood-response"
  "pet_robot_ros/LogMoodResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LogMood-response>)))
  "Returns md5sum for a message object of type '<LogMood-response>"
  "b43c763a2a7bf67b61c6235ef921779c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LogMood-response)))
  "Returns md5sum for a message object of type 'LogMood-response"
  "b43c763a2a7bf67b61c6235ef921779c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LogMood-response>)))
  "Returns full string definition for message of type '<LogMood-response>"
  (cl:format cl:nil "# Response~%bool success~%string response  # Supportive response from robot~%string[] suggestions  # Suggested coping activities~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LogMood-response)))
  "Returns full string definition for message of type 'LogMood-response"
  (cl:format cl:nil "# Response~%bool success~%string response  # Supportive response from robot~%string[] suggestions  # Suggested coping activities~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LogMood-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'response))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'suggestions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LogMood-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LogMood-response
    (cl:cons ':success (success msg))
    (cl:cons ':response (response msg))
    (cl:cons ':suggestions (suggestions msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LogMood)))
  'LogMood-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LogMood)))
  'LogMood-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogMood)))
  "Returns string type for a service object of type '<LogMood>"
  "pet_robot_ros/LogMood")
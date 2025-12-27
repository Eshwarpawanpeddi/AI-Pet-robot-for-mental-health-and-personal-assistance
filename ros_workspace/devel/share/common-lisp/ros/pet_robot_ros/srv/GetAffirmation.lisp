; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-srv)


;//! \htmlinclude GetAffirmation-request.msg.html

(cl:defclass <GetAffirmation-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetAffirmation-request (<GetAffirmation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAffirmation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAffirmation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<GetAffirmation-request> is deprecated: use pet_robot_ros-srv:GetAffirmation-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAffirmation-request>) ostream)
  "Serializes a message object of type '<GetAffirmation-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAffirmation-request>) istream)
  "Deserializes a message object of type '<GetAffirmation-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAffirmation-request>)))
  "Returns string type for a service object of type '<GetAffirmation-request>"
  "pet_robot_ros/GetAffirmationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAffirmation-request)))
  "Returns string type for a service object of type 'GetAffirmation-request"
  "pet_robot_ros/GetAffirmationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAffirmation-request>)))
  "Returns md5sum for a message object of type '<GetAffirmation-request>"
  "176b3ebe98603e26ffc647c2f65b7d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAffirmation-request)))
  "Returns md5sum for a message object of type 'GetAffirmation-request"
  "176b3ebe98603e26ffc647c2f65b7d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAffirmation-request>)))
  "Returns full string definition for message of type '<GetAffirmation-request>"
  (cl:format cl:nil "# Request a positive affirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAffirmation-request)))
  "Returns full string definition for message of type 'GetAffirmation-request"
  (cl:format cl:nil "# Request a positive affirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAffirmation-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAffirmation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAffirmation-request
))
;//! \htmlinclude GetAffirmation-response.msg.html

(cl:defclass <GetAffirmation-response> (roslisp-msg-protocol:ros-message)
  ((affirmation
    :reader affirmation
    :initarg :affirmation
    :type cl:string
    :initform "")
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0))
)

(cl:defclass GetAffirmation-response (<GetAffirmation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAffirmation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAffirmation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-srv:<GetAffirmation-response> is deprecated: use pet_robot_ros-srv:GetAffirmation-response instead.")))

(cl:ensure-generic-function 'affirmation-val :lambda-list '(m))
(cl:defmethod affirmation-val ((m <GetAffirmation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:affirmation-val is deprecated.  Use pet_robot_ros-srv:affirmation instead.")
  (affirmation m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <GetAffirmation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-srv:timestamp-val is deprecated.  Use pet_robot_ros-srv:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAffirmation-response>) ostream)
  "Serializes a message object of type '<GetAffirmation-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'affirmation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'affirmation))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAffirmation-response>) istream)
  "Deserializes a message object of type '<GetAffirmation-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'affirmation) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'affirmation) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAffirmation-response>)))
  "Returns string type for a service object of type '<GetAffirmation-response>"
  "pet_robot_ros/GetAffirmationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAffirmation-response)))
  "Returns string type for a service object of type 'GetAffirmation-response"
  "pet_robot_ros/GetAffirmationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAffirmation-response>)))
  "Returns md5sum for a message object of type '<GetAffirmation-response>"
  "176b3ebe98603e26ffc647c2f65b7d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAffirmation-response)))
  "Returns md5sum for a message object of type 'GetAffirmation-response"
  "176b3ebe98603e26ffc647c2f65b7d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAffirmation-response>)))
  "Returns full string definition for message of type '<GetAffirmation-response>"
  (cl:format cl:nil "# Response~%string affirmation~%time timestamp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAffirmation-response)))
  "Returns full string definition for message of type 'GetAffirmation-response"
  (cl:format cl:nil "# Response~%string affirmation~%time timestamp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAffirmation-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'affirmation))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAffirmation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAffirmation-response
    (cl:cons ':affirmation (affirmation msg))
    (cl:cons ':timestamp (timestamp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAffirmation)))
  'GetAffirmation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAffirmation)))
  'GetAffirmation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAffirmation)))
  "Returns string type for a service object of type '<GetAffirmation>"
  "pet_robot_ros/GetAffirmation")
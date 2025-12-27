; Auto-generated. Do not edit!


(cl:in-package pet_robot_ros-msg)


;//! \htmlinclude BreathingExerciseFeedback.msg.html

(cl:defclass <BreathingExerciseFeedback> (roslisp-msg-protocol:ros-message)
  ((current_step
    :reader current_step
    :initarg :current_step
    :type cl:integer
    :initform 0)
   (instruction
    :reader instruction
    :initarg :instruction
    :type cl:string
    :initform "")
   (progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0))
)

(cl:defclass BreathingExerciseFeedback (<BreathingExerciseFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BreathingExerciseFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BreathingExerciseFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pet_robot_ros-msg:<BreathingExerciseFeedback> is deprecated: use pet_robot_ros-msg:BreathingExerciseFeedback instead.")))

(cl:ensure-generic-function 'current_step-val :lambda-list '(m))
(cl:defmethod current_step-val ((m <BreathingExerciseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:current_step-val is deprecated.  Use pet_robot_ros-msg:current_step instead.")
  (current_step m))

(cl:ensure-generic-function 'instruction-val :lambda-list '(m))
(cl:defmethod instruction-val ((m <BreathingExerciseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:instruction-val is deprecated.  Use pet_robot_ros-msg:instruction instead.")
  (instruction m))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <BreathingExerciseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pet_robot_ros-msg:progress-val is deprecated.  Use pet_robot_ros-msg:progress instead.")
  (progress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BreathingExerciseFeedback>) ostream)
  "Serializes a message object of type '<BreathingExerciseFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'current_step)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'instruction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'instruction))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BreathingExerciseFeedback>) istream)
  "Deserializes a message object of type '<BreathingExerciseFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_step) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'instruction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'instruction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BreathingExerciseFeedback>)))
  "Returns string type for a message object of type '<BreathingExerciseFeedback>"
  "pet_robot_ros/BreathingExerciseFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BreathingExerciseFeedback)))
  "Returns string type for a message object of type 'BreathingExerciseFeedback"
  "pet_robot_ros/BreathingExerciseFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BreathingExerciseFeedback>)))
  "Returns md5sum for a message object of type '<BreathingExerciseFeedback>"
  "435a434729139d5d8aed2b71a0790f17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BreathingExerciseFeedback)))
  "Returns md5sum for a message object of type 'BreathingExerciseFeedback"
  "435a434729139d5d8aed2b71a0790f17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BreathingExerciseFeedback>)))
  "Returns full string definition for message of type '<BreathingExerciseFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback~%int32 current_step  # Current step in exercise~%string instruction  # Current instruction~%float32 progress  # 0.0 to 1.0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BreathingExerciseFeedback)))
  "Returns full string definition for message of type 'BreathingExerciseFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback~%int32 current_step  # Current step in exercise~%string instruction  # Current instruction~%float32 progress  # 0.0 to 1.0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BreathingExerciseFeedback>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'instruction))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BreathingExerciseFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'BreathingExerciseFeedback
    (cl:cons ':current_step (current_step msg))
    (cl:cons ':instruction (instruction msg))
    (cl:cons ':progress (progress msg))
))
